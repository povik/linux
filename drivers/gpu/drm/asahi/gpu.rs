// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

use core::any::Any;
use core::mem;
use core::sync::atomic::{AtomicU64, Ordering};
use core::time::Duration;

use kernel::{
    delay::coarse_sleep,
    macros::versions,
    prelude::*,
    soc::apple::rtkit,
    sync::{smutex::Mutex, Arc, Guard, UniqueArc},
    PointerWrapper,
};

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::channels::{DeviceControlMsg, FwCtlMsg, PipeType};
use crate::{alloc, buffer, channel, event, fw, gem, hw, initdata, mmu, render, workqueue};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gpu;

const EP_FIRMWARE: u8 = 0x20;
const EP_DOORBELL: u8 = 0x21;

const MSG_INIT: u64 = 0x81 << 48;
const INIT_DATA_MASK: u64 = (1 << 44) - 1;

const MSG_TX_DOORBELL: u64 = 0x83 << 48;
const MSG_FWCTL: u64 = 0x84 << 48;
const MSG_HALT: u64 = 0x85 << 48;

const MSG_RX_DOORBELL: u64 = 0x42 << 48;

const DOORBELL_KICKFW: u64 = 0x10;
const DOORBELL_DEVCTRL: u64 = 0x11;

pub(crate) struct KernelAllocators {
    pub(crate) private: alloc::SimpleAllocator,
    pub(crate) shared: alloc::SimpleAllocator,
    pub(crate) gpu: alloc::SimpleAllocator,
}

#[versions(AGX)]
struct RxChannels {
    event: channel::EventChannel,
    fw_log: channel::FwLogChannel,
    ktrace: channel::KTraceChannel,
    stats: channel::StatsChannel::ver,
}

struct PipeChannels {
    pub(crate) vtx: Vec<Mutex<channel::PipeChannel>>,
    pub(crate) frag: Vec<Mutex<channel::PipeChannel>>,
    pub(crate) comp: Vec<Mutex<channel::PipeChannel>>,
}

struct TxChannels {
    pub(crate) device_control: channel::DeviceControlChannel,
}

const NUM_PIPES: usize = 4;

pub(crate) struct ID(AtomicU64);

impl ID {
    pub(crate) fn new(val: u64) -> ID {
        ID(AtomicU64::new(val))
    }

    pub(crate) fn next(&self) -> u64 {
        self.0.fetch_add(1, Ordering::Relaxed)
    }
}

impl Default for ID {
    fn default() -> Self {
        ID(AtomicU64::new(2))
    }
}

#[derive(Default)]
pub(crate) struct SequenceIDs {
    pub(crate) file: ID,
    pub(crate) vm: ID,
    pub(crate) buf: ID,
    pub(crate) submission: ID,
    pub(crate) renderer: ID,
}

#[versions(AGX)]
pub(crate) struct GpuManager {
    dev: AsahiDevice,
    initialized: bool,
    pub(crate) initdata: fw::types::GpuObject<fw::initdata::InitData::ver>,
    uat: mmu::Uat,
    alloc: Mutex<KernelAllocators>,
    io_mappings: Vec<mmu::Mapping>,
    rtkit: Mutex<Option<rtkit::RTKit<GpuManager::ver>>>,
    rx_channels: Mutex<RxChannels::ver>,
    tx_channels: Mutex<TxChannels>,
    fwctl_channel: Mutex<channel::FwCtlChannel>,
    pipes: PipeChannels,
    event_manager: Arc<event::EventManager>,
    buffer_mgr: buffer::BufferManager,
    ids: SequenceIDs,
}

pub(crate) trait GpuManager: Send + Sync {
    fn as_any(&self) -> &dyn Any;
    fn init(&self) -> Result;
    fn update_globals(&self);
    fn alloc(&self) -> Guard<'_, Mutex<KernelAllocators>>;
    fn new_vm(&self) -> Result<mmu::Vm>;
    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind>;
    fn new_renderer(
        &self,
        ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::SimpleAllocator>>,
    ) -> Result<Box<dyn render::Renderer>>;
    fn submit_batch(&self, batch: workqueue::WorkQueueBatch<'_>) -> Result;
    fn ids(&self) -> &SequenceIDs;
    fn kick_firmware(&self) -> Result;
    fn invalidate_context(
        &self,
        context: &fw::types::GpuObject<fw::workqueue::GpuContextData>,
    ) -> Result;
    fn handle_timeout(&self, counter: u32, event_slot: u32);
    fn handle_fault(&self);
    fn wait_for_poweroff(&self, timeout: usize) -> Result;
    fn fwctl(&self, msg: FwCtlMsg) -> Result;
}

#[versions(AGX)]
#[vtable]
impl rtkit::Operations for GpuManager::ver {
    type Data = Arc<GpuManager::ver>;
    type Buffer = gem::ObjectRef;

    fn recv_message(data: <Self::Data as PointerWrapper>::Borrowed<'_>, ep: u8, msg: u64) {
        let dev = &data.dev;
        //dev_info!(dev, "RTKit message: {:#x}:{:#x}\n", ep, msg);

        if ep != EP_FIRMWARE || msg != MSG_RX_DOORBELL {
            dev_err!(dev, "Unknown message: {:#x}:{:#x}\n", ep, msg);
            return;
        }

        let mut ch = data.rx_channels.lock();

        ch.fw_log.poll();
        ch.ktrace.poll();
        ch.stats.poll();
        ch.event.poll();
    }

    fn shmem_alloc(
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        size: usize,
    ) -> Result<Self::Buffer> {
        let dev = &data.dev;
        mod_dev_dbg!(dev, "shmem_alloc() {:#x} bytes\n", size);

        let mut obj = gem::new_kernel_object(dev, size)?;
        obj.vmap()?;
        let iova = obj.map_into(data.uat.kernel_vm())?;
        mod_dev_dbg!(dev, "shmem_alloc() -> VA {:#x}\n", iova);
        Ok(obj)
    }
}

#[versions(AGX)]
impl GpuManager::ver {
    pub(crate) fn new(dev: &AsahiDevice, cfg: &hw::HwConfig) -> Result<Arc<GpuManager::ver>> {
        let uat = mmu::Uat::new(dev)?;
        let mut alloc = KernelAllocators {
            private: alloc::SimpleAllocator::new(dev, uat.kernel_vm(), 0x20, mmu::PROT_FW_PRIV_RW),
            shared: alloc::SimpleAllocator::new(dev, uat.kernel_vm(), 0x20, mmu::PROT_FW_SHARED_RW),
            gpu: alloc::SimpleAllocator::new(
                dev,
                uat.kernel_vm(),
                0x20,
                mmu::PROT_GPU_FW_SHARED_RW,
            ),
        };

        let dyncfg = GpuManager::ver::get_dyn_config(dev, &uat, cfg)?;

        let mut builder = initdata::InitDataBuilder::ver::new(&mut alloc, cfg, &dyncfg);
        let initdata = builder.build()?;

        let mut pipes = PipeChannels {
            vtx: Vec::new(),
            frag: Vec::new(),
            comp: Vec::new(),
        };

        for _i in 0..=NUM_PIPES - 1 {
            pipes
                .vtx
                .try_push(Mutex::new(channel::PipeChannel::new(&mut alloc)?))?;
            pipes
                .frag
                .try_push(Mutex::new(channel::PipeChannel::new(&mut alloc)?))?;
            pipes
                .comp
                .try_push(Mutex::new(channel::PipeChannel::new(&mut alloc)?))?;
        }

        let event_manager = Arc::try_new(event::EventManager::new(&mut alloc)?)?;

        let mut mgr = UniqueArc::try_new(GpuManager::ver {
            dev: dev.clone(),
            initialized: false,
            initdata,
            uat,
            io_mappings: Vec::new(),
            rtkit: Mutex::new(None),
            rx_channels: Mutex::new(RxChannels::ver {
                event: channel::EventChannel::new(&mut alloc, event_manager.clone())?,
                fw_log: channel::FwLogChannel::new(&mut alloc)?,
                ktrace: channel::KTraceChannel::new(&mut alloc)?,
                stats: channel::StatsChannel::ver::new(&mut alloc)?,
            }),
            tx_channels: Mutex::new(TxChannels {
                device_control: channel::DeviceControlChannel::new(&mut alloc)?,
            }),
            fwctl_channel: Mutex::new(channel::FwCtlChannel::new(&mut alloc)?),
            pipes,
            event_manager,
            buffer_mgr: buffer::BufferManager::new()?,
            alloc: Mutex::new(alloc),
            ids: Default::default(),
        })?;

        {
            let fwctl = mgr.fwctl_channel.lock();
            let p_fwctl = fwctl.to_raw();
            mem::drop(fwctl);

            mgr.initdata.fw_status.with_mut(|raw, _inner| {
                raw.fwctl_channel = p_fwctl;
            });
        }

        {
            let txc = mgr.tx_channels.lock();
            let p_device_control = txc.device_control.to_raw();
            mem::drop(txc);

            let rxc = mgr.rx_channels.lock();
            let p_event = rxc.event.to_raw();
            let p_fw_log = rxc.fw_log.to_raw();
            let p_ktrace = rxc.ktrace.to_raw();
            let p_stats = rxc.stats.to_raw();
            mem::drop(rxc);

            mgr.initdata.runtime_pointers.with_mut(|raw, _inner| {
                raw.device_control = p_device_control;
                raw.event = p_event;
                raw.fw_log = p_fw_log;
                raw.ktrace = p_ktrace;
                raw.stats = p_stats;
            });
        }

        let mut p_pipes: Vec<fw::initdata::raw::PipeChannels> = Vec::new();

        for ((v, f), c) in mgr
            .pipes
            .vtx
            .iter()
            .zip(&mgr.pipes.frag)
            .zip(&mgr.pipes.comp)
        {
            p_pipes.try_push(fw::initdata::raw::PipeChannels {
                vtx: v.lock().to_raw(),
                frag: f.lock().to_raw(),
                comp: c.lock().to_raw(),
            })?;
        }

        mgr.initdata.runtime_pointers.with_mut(|raw, _inner| {
            for (i, p) in p_pipes.into_iter().enumerate() {
                raw.pipes[i].vtx = p.vtx;
                raw.pipes[i].frag = p.frag;
                raw.pipes[i].comp = p.comp;
            }
        });

        for (i, map) in cfg.io_mappings.iter().enumerate() {
            if let Some(map) = map.as_ref() {
                mgr.iomap(i, map)?;
            }
        }

        let mgr = Arc::from(mgr);

        let rtkit = rtkit::RTKit::<GpuManager::ver>::new(dev, None, 0, mgr.clone())?;

        *mgr.rtkit.lock() = Some(rtkit);

        {
            let mut rxc = mgr.rx_channels.lock();
            rxc.event.set_manager(mgr.clone());
        }

        Ok(mgr)
    }

    fn get_dyn_config(
        dev: &AsahiDevice,
        uat: &mmu::Uat,
        cfg: &hw::HwConfig,
    ) -> Result<hw::DynConfig> {
        Ok(hw::DynConfig {
            pwr: hw::PwrConfig::load(dev, cfg)?,
            uat_ttb_base: uat.ttb_base(),
        })
    }

    fn iomap(&mut self, index: usize, map: &hw::IOMapping) -> Result {
        let off = map.base & mmu::UAT_PGMSK;
        let base = map.base - off;
        let end = (map.base + map.size + mmu::UAT_PGMSK) & !mmu::UAT_PGMSK;
        let mapping = self
            .uat
            .kernel_vm()
            .map_io(base, end - base, map.writable)?;

        self.initdata.runtime_pointers.hwdata_b.with_mut(|raw, _| {
            raw.io_mappings[index] = fw::initdata::raw::IOMapping {
                phys_addr: map.base as u64,
                virt_addr: (mapping.iova() + off) as u64,
                size: map.size as u32,
                range_size: map.range_size as u32,
                readwrite: map.writable as u64,
            };
        });

        self.io_mappings.try_push(mapping)?;
        Ok(())
    }

    fn show_pending_events(&self) {
        dev_err!(self.dev, "  Pending events:\n");

        self.initdata.globals.with(|raw, _inner| {
            for i in raw.pending_stamps.iter() {
                let info = i.info.load(Ordering::Relaxed);
                let wait_value = i.wait_value.load(Ordering::Relaxed);

                if info != 0 {
                    let slot = info >> 3;
                    let flags = info & 0x7;
                    dev_err!(
                        self.dev,
                        "    [{}] flags={} value={:#x}\n",
                        slot,
                        flags,
                        wait_value
                    );
                    i.info.store(0, Ordering::Relaxed);
                    i.wait_value.store(0, Ordering::Relaxed);
                }
            }
        });
    }

    fn show_fault_info(&self) {
        let data = self.dev.data();

        let res = match data.resources() {
            Some(res) => res,
            None => {
                dev_err!(self.dev, "  Failed to acquire resources\n");
                return;
            }
        };

        if let Some(info) = res.get_fault_info() {
            dev_err!(self.dev, "  Fault info: {:#x?}\n", info);
        }
    }

    fn recover(&self) {
        self.initdata.fw_status.with(|raw, _inner| {
            let halt_count = raw.flags.halt_count.load(Ordering::Relaxed);
            let halted = raw.flags.halted.load(Ordering::Relaxed);
            dev_err!(self.dev, "  Halt count: {}\n", halt_count);
            dev_err!(self.dev, "  Halted: {}\n", halted);

            if debug_enabled(DebugFlags::NoGpuRecovery) {
                dev_crit!(self.dev, "  GPU recovery is disabled, wedging forever!\n");
            } else if halted != 0 {
                dev_err!(self.dev, "  Attempting recovery...\n");
                raw.flags.halted.store(0, Ordering::SeqCst);
                raw.flags.resume.store(1, Ordering::SeqCst);
            } else {
                dev_err!(self.dev, "  Cannot recover.\n");
            }
        });
    }
}

#[versions(AGX)]
impl GpuManager for GpuManager::ver {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn init(&self) -> Result {
        self.tx_channels
            .lock()
            .device_control
            .send(&DeviceControlMsg::Initialize(Default::default()));

        let initdata = self.initdata.gpu_va().get();
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();

        rtk.boot()?;
        rtk.start_endpoint(EP_FIRMWARE)?;
        rtk.start_endpoint(EP_DOORBELL)?;
        rtk.send_message(EP_FIRMWARE, MSG_INIT | (initdata & INIT_DATA_MASK))?;
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        Ok(())
    }

    fn update_globals(&self) {
        let mut timeout: u32 = 2;
        if debug_enabled(DebugFlags::WaitForPowerOff) {
            timeout = 0;
        } else if debug_enabled(DebugFlags::KeepGpuPowered) {
            timeout = 5000;
        }

        self.initdata.globals.with(|raw, _inner| {
            raw.idle_to_off_timeout_ms.store(timeout, Ordering::Relaxed);
        });
    }

    fn alloc(&self) -> Guard<'_, Mutex<KernelAllocators>> {
        self.alloc.lock()
    }

    fn new_vm(&self) -> Result<mmu::Vm> {
        self.uat.new_vm(self.ids.vm.next())
    }

    fn bind_vm(&self, vm: &mmu::Vm) -> Result<mmu::VmBind> {
        self.uat.bind(vm)
    }

    fn new_renderer(
        &self,
        ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::SimpleAllocator>>,
    ) -> Result<Box<dyn render::Renderer>> {
        let mut kalloc = self.alloc();
        let id = self.ids.renderer.next();
        Ok(Box::try_new(render::Renderer::ver::new(
            &self.dev,
            &mut *kalloc,
            ualloc,
            ualloc_priv,
            self.event_manager.clone(),
            &self.buffer_mgr,
            id,
        )?)?)
    }

    fn submit_batch(&self, batch: workqueue::WorkQueueBatch<'_>) -> Result {
        let pipe_type = batch.pipe_type();
        let pipes = match pipe_type {
            PipeType::Vertex => &self.pipes.vtx,
            PipeType::Fragment => &self.pipes.frag,
            PipeType::Compute => &self.pipes.comp,
        };

        /* TODO: need try_lock()
        let pipe = 'outer: loop {
            for p in pipes.iter() {
                if let Ok(guard) = p.try_lock() {
                    break 'outer guard;
                }
            }
            break pipes[0].lock();
        };
        */

        let index: usize = 0;
        let mut pipe = pipes[index].lock();

        batch.submit(&mut pipe)?;

        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();
        rtk.send_message(
            EP_DOORBELL,
            MSG_TX_DOORBELL | pipe_type as u64 | ((index as u64) << 2),
        )?;

        Ok(())
    }

    fn kick_firmware(&self) -> Result {
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().unwrap();
        rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_KICKFW)?;

        Ok(())
    }

    fn invalidate_context(
        &self,
        context: &fw::types::GpuObject<fw::workqueue::GpuContextData>,
    ) -> Result {
        mod_dev_dbg!(
            self.dev,
            "Invalidating GPU context @ {:?}\n",
            context.weak_pointer()
        );

        let dc = context.with(|raw, _inner| DeviceControlMsg::DestroyContext {
            unk_4: 0,
            ctx_23: raw.unk_23,
            unk_c: 0,
            unk_10: 0,
            ctx_0: raw.unk_0,
            ctx_1: raw.unk_1,
            ctx_4: raw.unk_4,
            unk_18: 0,
            gpu_context: context.weak_pointer(),
            __pad: Default::default(),
        });

        mod_dev_dbg!(self.dev, "Context invalidation command: {:?}\n", &dc);

        let mut txch = self.tx_channels.lock();

        let token = txch.device_control.send(&dc);

        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_TX_DOORBELL | DOORBELL_DEVCTRL)?;
        }

        txch.device_control.wait_for(token)?;

        mod_dev_dbg!(
            self.dev,
            "GPU context invalidated: {:?}\n",
            context.weak_pointer()
        );

        Ok(())
    }

    fn ids(&self) -> &SequenceIDs {
        &self.ids
    }

    fn handle_timeout(&self, counter: u32, event_slot: u32) {
        dev_err!(self.dev, " (\\________/) \n");
        dev_err!(self.dev, "  |        |  \n");
        dev_err!(self.dev, "'.| \\  , / |.'\n");
        dev_err!(self.dev, "--| / (( \\ |--\n");
        dev_err!(self.dev, ".'|  _-_-  |'.\n");
        dev_err!(self.dev, "  |________|  \n");
        dev_err!(self.dev, "** GPU timeout nya~!!!!! **\n");
        dev_err!(self.dev, "  Event slot: {}\n", event_slot);
        dev_err!(self.dev, "  Timeout count: {}\n", counter);
        self.show_pending_events();
        self.show_fault_info();
        self.recover();
    }

    fn handle_fault(&self) {
        dev_err!(self.dev, " (\\________/) \n");
        dev_err!(self.dev, "  |        |  \n");
        dev_err!(self.dev, "'.| \\  , / |.'\n");
        dev_err!(self.dev, "--| / (( \\ |--\n");
        dev_err!(self.dev, ".'|  _-_-  |'.\n");
        dev_err!(self.dev, "  |________|  \n");
        dev_err!(self.dev, "GPU fault nya~!!!!!\n");
        self.show_pending_events();
        self.show_fault_info();
        self.recover();
    }

    fn wait_for_poweroff(&self, timeout: usize) -> Result {
        self.initdata.runtime_pointers.hwdata_a.with(|raw, _inner| {
            for _i in 0..timeout {
                if raw.pwr_status.load(Ordering::Relaxed) == 4 {
                    return Ok(());
                }
                coarse_sleep(Duration::from_millis(1));
            }
            Err(ETIMEDOUT)
        })
    }

    fn fwctl(&self, msg: fw::channels::FwCtlMsg) -> Result {
        let mut fwctl = self.fwctl_channel.lock();
        let token = fwctl.send(&msg);
        {
            let mut guard = self.rtkit.lock();
            let rtk = guard.as_mut().unwrap();
            rtk.send_message(EP_DOORBELL, MSG_FWCTL)?;
        }
        fwctl.wait_for(token)?;
        Ok(())
    }
}

#[versions(AGX)]
unsafe impl Sync for GpuManager::ver {}

#[versions(AGX)]
unsafe impl Send for GpuManager::ver {}
