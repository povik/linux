// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Apple AGX UAT (MMU) support

use core::arch::asm;
use core::fmt::Debug;
use core::mem::{size_of, ManuallyDrop};
use core::ops::Deref;
use core::ptr::NonNull;
use core::sync::atomic::{fence, AtomicU32, AtomicU64, AtomicU8, Ordering};

use kernel::{
    bindings, c_str, device,
    drm::{gem, gem::shmem, mm},
    error::{to_result, Result},
    io_pgtable,
    io_pgtable::{prot, AppleUAT, AppleUATCfg, IOPagetable},
    prelude::*,
    str::CString,
    sync::smutex::Mutex,
    sync::Arc,
    PointerWrapper,
};

use crate::no_debug;
use crate::slotalloc;

const PPL_MAGIC: u64 = 0x4b1d000000000002;

const UAT_NUM_CTX: usize = 64;
const UAT_USER_CTX_START: usize = 1;
const UAT_USER_CTX: usize = UAT_NUM_CTX - UAT_USER_CTX_START;

pub(crate) const UAT_PGBIT: usize = 14;
pub(crate) const UAT_PGSZ: usize = 1 << UAT_PGBIT;
pub(crate) const UAT_PGMSK: usize = UAT_PGSZ - 1;

type Pte = AtomicU64;
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

pub(crate) const UAT_IAS: usize = 39;
pub(crate) const UAT_IAS_KERN: usize = 36;
pub(crate) const UAT_OAS: usize = 36;

const IOVA_USER_BASE: usize = UAT_PGSZ;
const IOVA_USER_TOP: usize = (1 << UAT_IAS) - 1;
const IOVA_TTBR1_BASE: usize = 0xffffff8000000000;
const IOVA_KERN_BASE: usize = 0xffffffa000000000;
const IOVA_KERN_TOP: usize = 0xffffffafffffffff;

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

const PTE_TABLE: u64 = 0x3; // BIT(0) | BIT(1)

pub(crate) const PROT_FW_SHARED_RW: u32 = prot::PRIV | prot::READ | prot::WRITE | prot::CACHE;
pub(crate) const PROT_FW_SHARED_RO: u32 = prot::PRIV | prot::READ | prot::CACHE;
pub(crate) const PROT_FW_PRIV_RW: u32 = prot::PRIV | prot::READ | prot::WRITE;
pub(crate) const PROT_FW_PRIV_RO: u32 = prot::PRIV | prot::READ;
pub(crate) const PROT_GPU_FW_SHARED_RW: u32 = prot::READ | prot::WRITE | prot::CACHE;
pub(crate) const PROT_GPU_SHARED_RW: u32 = prot::READ | prot::WRITE | prot::CACHE | prot::NOEXEC;
pub(crate) const PROT_GPU_SHARED_RO: u32 = prot::READ | prot::CACHE | prot::NOEXEC;
pub(crate) const PROT_GPU_PRIV_RW: u32 = prot::READ | prot::WRITE | prot::NOEXEC;
pub(crate) const PROT_GPU_PRIV_RO: u32 = prot::READ | prot::NOEXEC;

type PhysAddr = bindings::phys_addr_t;

struct UatRegion {
    base: PhysAddr,
    size: usize,
    map: NonNull<core::ffi::c_void>,
}

#[repr(C)]
struct FlushInfo {
    state: AtomicU64,
    addr: AtomicU64,
    size: AtomicU64,
}

#[repr(C)]
struct Handoff {
    magic_ap: AtomicU64,
    magic_fw: AtomicU64,

    lock_ap: AtomicU8,
    lock_fw: AtomicU8,
    // Implicit padding: 2 bytes
    turn: AtomicU32,
    unk: AtomicU32,
    // Implicit padding: 4 bytes
    flush: [FlushInfo; UAT_NUM_CTX + 1],

    unk2: AtomicU8,
    // Implicit padding: 7 bytes
    unk3: AtomicU64,
}

const HANDOFF_SIZE: usize = size_of::<Handoff>();

#[repr(C)]
struct SlotTTBS {
    ttb0: AtomicU64,
    ttb1: AtomicU64,
}

const SLOTS_SIZE: usize = UAT_NUM_CTX * size_of::<SlotTTBS>();

// We need at least page 0 (ttb0)
const PAGETABLES_SIZE: usize = UAT_PGSZ;

struct VmInner {
    dev: device::Device,
    is_kernel: bool,
    min_va: usize,
    max_va: usize,
    page_table: IOPagetable<Uat, AppleUATCfg>,
    mm: mm::Allocator<MappingInner>,
    uat_inner: Arc<Mutex<UatInner>>,
    active_users: usize,
    binding: Option<slotalloc::Guard<SlotInner>>,
    bind_token: Option<slotalloc::SlotToken>,
    id: u64,
}

impl VmInner {
    fn ttb(&self) -> u64 {
        self.page_table.cfg().ttbr
    }

    fn map_iova(&self, iova: usize, size: usize) -> Result<usize> {
        if iova < self.min_va || (iova + size - 1) > self.max_va {
            Err(EINVAL)
        } else if self.is_kernel {
            Ok(iova - self.min_va)
        } else {
            Ok(iova)
        }
    }

    fn map_pages(
        &mut self,
        mut iova: usize,
        mut paddr: usize,
        pgsize: usize,
        pgcount: usize,
        prot: u32,
    ) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let mapped = self.page_table.map_pages(
                self.map_iova(iova, pgsize * pgcount)?,
                paddr,
                pgsize,
                left,
                prot,
            )?;
            assert!(mapped <= left * pgsize);

            left -= mapped / pgsize;
            paddr += mapped;
            iova += mapped;
        }
        Ok(pgcount * pgsize)
    }

    fn unmap_pages(&mut self, mut iova: usize, pgsize: usize, pgcount: usize) -> Result<usize> {
        let mut left = pgcount;
        while left > 0 {
            let unmapped = self.page_table.unmap_pages(
                self.map_iova(iova, pgsize * pgcount)?,
                pgsize,
                pgcount,
            );
            assert!(unmapped <= left * pgsize);

            left -= unmapped / pgsize;
            iova += unmapped;
        }
        Ok(pgcount * pgsize)
    }
}

#[derive(Clone)]
pub(crate) struct Vm {
    inner: Arc<Mutex<VmInner>>,
}
no_debug!(Vm);

pub(crate) struct SlotInner();

impl slotalloc::SlotItem for SlotInner {
    type Owner = ();
}

#[derive(Debug)]
pub(crate) struct VmBind(Vm, u32);

impl VmBind {
    pub(crate) fn slot(&self) -> u32 {
        self.1
    }
}

impl Drop for VmBind {
    fn drop(&mut self) {
        let mut inner = self.0.inner.lock();

        assert_ne!(inner.active_users, 0);
        inner.active_users -= 1;
        if inner.active_users == 0 {
            inner.binding = None;
        }
    }
}

impl Clone for VmBind {
    fn clone(&self) -> VmBind {
        let mut inner = self.0.inner.lock();

        inner.active_users += 1;
        VmBind(self.0.clone(), self.1)
    }
}

pub(crate) struct MappingInner {
    owner: Arc<Mutex<VmInner>>,
}

pub(crate) struct Mapping(mm::Node<MappingInner>);

impl Mapping {
    pub(crate) fn iova(&self) -> usize {
        self.0.start() as usize
    }
    pub(crate) fn size(&self) -> usize {
        self.0.size() as usize
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        let mut owner = self.0.owner.lock();
        dev_info!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}",
            self.iova(),
            self.size()
        );
        // Do not try to unmap guard page (-1)
        if owner
            .unmap_pages(self.iova(), UAT_PGSZ, (self.size() >> UAT_PGBIT) - 1)
            .is_err()
        {
            dev_err!(
                owner.dev,
                "MMU: unmap {:#x}:{:#x} failed",
                self.iova(),
                self.size()
            );
        }

        unsafe {
            asm!(".arch armv8.4-a\ndsb sy\ntlbi vmalle1os\ndsb sy\n");
        }
    }
}

struct UatInner {
    handoff_rgn: UatRegion,
    ttbs_rgn: UatRegion,
}

impl UatInner {
    fn handoff(&self) -> &Handoff {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.handoff_rgn.map.as_ptr() as *mut Handoff).as_ref() }.unwrap()
    }

    fn ttbs(&self) -> &[SlotTTBS; UAT_NUM_CTX] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.ttbs_rgn.map.as_ptr() as *mut [SlotTTBS; UAT_NUM_CTX]).as_ref() }.unwrap()
    }
}

unsafe impl Send for UatInner {}

pub(crate) struct Uat {
    dev: device::Device,
    pagetables_rgn: UatRegion,

    inner: Arc<Mutex<UatInner>>,
    slots: slotalloc::SlotAllocator<SlotInner>,

    kernel_vm: Vm,
    kernel_lower_vm: Vm,
}

impl Drop for UatRegion {
    fn drop(&mut self) {
        // SAFETY: the pointer is valid by the type invariant
        unsafe { bindings::memunmap(self.map.as_ptr()) };
    }
}

impl Handoff {
    fn lock(&self) {
        self.lock_ap.store(1, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        while self.lock_fw.load(Ordering::Relaxed) != 0 {
            if self.turn.load(Ordering::Relaxed) != 0 {
                self.lock_ap.store(0, Ordering::Relaxed);
                while self.turn.load(Ordering::Relaxed) != 0 {}
                self.lock_ap.store(1, Ordering::Relaxed);
                fence(Ordering::SeqCst);
            }
        }
        fence(Ordering::Acquire);
    }

    fn unlock(&self) {
        self.turn.store(1, Ordering::Relaxed);
        self.lock_ap.store(0, Ordering::Release);
    }

    fn init(&self) {
        self.magic_ap.store(PPL_MAGIC, Ordering::Relaxed);
        self.unk.store(0xffffffff, Ordering::Relaxed);
        self.unk3.store(0, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        self.lock();

        while self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {}

        self.unlock();

        for i in 0..=UAT_NUM_CTX {
            self.flush[i].state.store(0, Ordering::Relaxed);
            self.flush[i].addr.store(0, Ordering::Relaxed);
            self.flush[i].size.store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
    }
}

impl io_pgtable::FlushOps for Uat {
    type Data = ();

    fn tlb_flush_all(_data: <Self::Data as PointerWrapper>::Borrowed<'_>) {
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
    fn tlb_flush_walk(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _size: usize,
        _granule: usize,
    ) {
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
    fn tlb_add_page(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _granule: usize,
    ) {
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
}

impl Vm {
    fn new(
        dev: device::Device,
        uat_inner: Arc<Mutex<UatInner>>,
        is_kernel: bool,
        id: u64,
    ) -> Result<Vm> {
        let page_table = AppleUAT::new(
            &dev,
            io_pgtable::Config {
                pgsize_bitmap: UAT_PGSZ,
                ias: if is_kernel { UAT_IAS_KERN } else { UAT_IAS },
                oas: UAT_OAS,
                coherent_walk: true,
                quirks: 0,
            },
            (),
        )?;
        let min_va = if is_kernel {
            IOVA_KERN_BASE
        } else {
            IOVA_USER_BASE
        };
        let max_va = if is_kernel {
            IOVA_KERN_TOP
        } else {
            IOVA_USER_TOP
        };

        let mm = mm::Allocator::new(min_va as u64, (max_va - min_va + 1) as u64)?;

        Ok(Vm {
            inner: Arc::try_new(Mutex::new(VmInner {
                dev,
                min_va,
                max_va,
                is_kernel,
                page_table,
                mm,
                uat_inner,
                binding: None,
                bind_token: None,
                active_users: 0,
                id,
            }))?,
        })
    }

    fn ttb(&self) -> u64 {
        self.inner.lock().ttb()
    }

    pub(crate) fn map(&self, size: usize, sgt: &mut shmem::SGTableIter<'_>) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let node = inner.mm.insert_node(
            MappingInner {
                owner: self.inner.clone(),
            },
            (size + UAT_PGSZ) as u64, // Add guard page
        )?;

        Self::map_node(
            &mut *inner,
            node,
            sgt,
            prot::PRIV | prot::READ | prot::WRITE | prot::CACHE,
        )
    }

    pub(crate) fn map_in_range(
        &self,
        size: usize,
        sgt: &mut shmem::SGTableIter<'_>,
        alignment: u64,
        start: u64,
        end: u64,
        prot: u32,
    ) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let node = inner.mm.insert_node_in_range(
            MappingInner {
                owner: self.inner.clone(),
            },
            (size + UAT_PGSZ) as u64, // Add guard page
            alignment,
            0,
            start,
            end,
            mm::InsertMode::Best,
        )?;

        Self::map_node(&mut *inner, node, sgt, prot)
    }

    fn map_node(
        inner: &mut VmInner,
        node: mm::Node<MappingInner>,
        sgt: &mut shmem::SGTableIter<'_>,
        prot: u32,
    ) -> Result<Mapping> {
        let mut iova = node.start() as usize;

        for range in sgt {
            let addr = range.dma_address();
            let len = range.dma_len();

            if (addr | len | iova) & UAT_PGMSK != 0 {
                dev_err!(
                    inner.dev,
                    "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned",
                    addr,
                    len,
                    iova
                );
                return Err(EINVAL);
            }

            dev_info!(inner.dev, "MMU: map: {:#x}:{:#x} -> {:#x}", addr, len, iova);

            inner.map_pages(iova, addr, UAT_PGSZ, len >> UAT_PGBIT, prot)?;

            iova += len;
        }

        unsafe {
            asm!(".arch armv8.4-a\ndsb sy\n");
        }

        Ok(Mapping(node))
    }

    pub(crate) fn map_io(&self, phys: usize, size: usize) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let node = inner.mm.insert_node(
            MappingInner {
                owner: self.inner.clone(),
            },
            (size + UAT_PGSZ) as u64, // Add guard page
        )?;

        let iova = node.start() as usize;

        if (phys | size | iova) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev,
                "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(
            inner.dev,
            "MMU: IO map: {:#x}:{:#x} -> {:#x}",
            phys,
            size,
            iova
        );

        inner.map_pages(
            iova,
            phys,
            UAT_PGSZ,
            size >> UAT_PGBIT,
            prot::PRIV | prot::READ | prot::WRITE | prot::CACHE | prot::MMIO,
        )?;

        Ok(Mapping(node))
    }
}

impl Drop for VmInner {
    fn drop(&mut self) {
        assert_eq!(self.active_users, 0);

        // Make sure this VM is not mapped to a TTB if it was
        if let Some(token) = self.bind_token.take() {
            let idx = (token.last_slot() as usize) + UAT_USER_CTX_START;
            let ttb = self.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

            let uat_inner = self.uat_inner.lock();
            uat_inner.handoff().lock();
            let inval = uat_inner.ttbs()[idx]
                .ttb0
                .compare_exchange(ttb, 0, Ordering::SeqCst, Ordering::Relaxed)
                .is_ok();
            uat_inner.handoff().unlock();
            core::mem::drop(uat_inner);

            if inval {
                // TODO: invalidate ASID only
                unsafe {
                    asm!(".arch armv8.4-a\ntlbi vmalle1os");
                }
            }
        }
    }
}

impl Uat {
    fn map_region(dev: &dyn device::RawDevice, name: &CStr, size: usize) -> Result<UatRegion> {
        let rdev = dev.raw_device();

        let mut res = core::mem::MaybeUninit::<bindings::resource>::uninit();

        let res = unsafe {
            let idx = bindings::of_property_match_string(
                (*rdev).of_node,
                c_str!("memory-region-names").as_char_ptr(),
                name.as_char_ptr(),
            );
            to_result(idx)?;

            let np = bindings::of_parse_phandle(
                (*rdev).of_node,
                c_str!("memory-region").as_char_ptr(),
                idx,
            );
            if np.is_null() {
                dev_err!(dev, "Missing {} region\n", name);
                return Err(EINVAL);
            }
            let ret = bindings::of_address_to_resource(np, 0, res.as_mut_ptr());
            bindings::of_node_put(np);

            if ret < 0 {
                dev_err!(dev, "Failed to get {} region\n", name);
                to_result(ret)?
            }

            res.assume_init()
        };

        let rgn_size: usize = unsafe { bindings::resource_size(&res) } as usize;

        if size > rgn_size {
            dev_err!(
                dev,
                "Region {} is too small (expected {}, got {})\n",
                name,
                size,
                rgn_size
            );
            return Err(ENOMEM);
        }

        let map = unsafe { bindings::memremap(res.start, rgn_size, bindings::MEMREMAP_WB.into()) };
        let map = NonNull::new(map);

        match map {
            None => {
                dev_err!(dev, "Failed to remap {} region\n", name);
                Err(ENOMEM)
            }
            Some(map) => Ok(UatRegion {
                base: res.start,
                size: rgn_size as usize,
                map,
            }),
        }
    }

    fn kpt0(&self) -> &[Pte; UAT_NPTE] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.pagetables_rgn.map.as_ptr() as *mut [Pte; UAT_NPTE]).as_ref() }.unwrap()
    }

    pub(crate) fn kernel_vm(&self) -> &Vm {
        &self.kernel_vm
    }

    pub(crate) fn context_table_base(&self) -> u64 {
        let inner = self.inner.lock();

        inner.ttbs_rgn.base as u64
    }

    pub(crate) fn bind(&self, vm: &Vm) -> Result<VmBind> {
        let mut inner = vm.inner.lock();

        if inner.binding.is_none() {
            assert_eq!(inner.active_users, 0);

            let slot = self.slots.get(inner.bind_token)?;
            if slot.changed() {
                let idx = (slot.slot() as usize) + UAT_USER_CTX_START;
                let ttb = inner.ttb() | TTBR_VALID | (idx as u64) << TTBR_ASID_SHIFT;

                let uat_inner = self.inner.lock();
                let ttbs = uat_inner.ttbs();
                uat_inner.handoff().lock();
                ttbs[idx].ttb0.store(ttb, Ordering::Relaxed);
                ttbs[idx].ttb1.store(0, Ordering::Relaxed);
                uat_inner.handoff().unlock();
            }

            inner.bind_token = Some(slot.token());
            inner.binding = Some(slot);
        }

        inner.active_users += 1;

        Ok(VmBind(
            vm.clone(),
            inner.binding.as_ref().unwrap().slot() + UAT_USER_CTX_START as u32,
        ))
    }

    pub(crate) fn new_vm(&self, id: u64) -> Result<Vm> {
        Vm::new(self.dev.clone(), self.inner.clone(), false, id)
    }

    pub(crate) fn new(dev: &dyn device::RawDevice) -> Result<Self> {
        dev_info!(dev, "MMU: Initializing...\n");

        let handoff_rgn = Self::map_region(dev, c_str!("handoff"), HANDOFF_SIZE)?;
        let ttbs_rgn = Self::map_region(dev, c_str!("ttbs"), SLOTS_SIZE)?;
        let pagetables_rgn = Self::map_region(dev, c_str!("pagetables"), PAGETABLES_SIZE)?;

        dev_info!(dev, "MMU: Initializing kernel page table\n");

        let inner = Arc::try_new(Mutex::new(UatInner {
            handoff_rgn,
            ttbs_rgn,
        }))?;

        let kernel_lower_vm = Vm::new(device::Device::from_dev(dev), inner.clone(), false, 1)?;
        let kernel_vm = Vm::new(device::Device::from_dev(dev), inner.clone(), true, 0)?;

        let ttb0 = kernel_lower_vm.ttb();
        let ttb1 = kernel_vm.ttb();

        let uat = Self {
            dev: device::Device::from_dev(dev),
            pagetables_rgn,
            kernel_vm,
            kernel_lower_vm,
            inner,
            slots: slotalloc::SlotAllocator::new(UAT_USER_CTX as u32, (), |_inner, _slot| {
                SlotInner()
            })?,
        };

        let inner = uat.inner.lock();

        inner.handoff().init();

        dev_info!(dev, "MMU: Initializing TTBs\n");

        inner.handoff().lock();

        let ttbs = inner.ttbs();

        ttbs[0].ttb0.store(ttb0 | TTBR_VALID, Ordering::Relaxed);
        ttbs[0]
            .ttb1
            .store(uat.pagetables_rgn.base | TTBR_VALID, Ordering::Relaxed);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        inner.handoff().unlock();

        core::mem::drop(inner);

        uat.kpt0()[2].store(ttb1 | PTE_TABLE, Ordering::Relaxed);

        dev_info!(dev, "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        // Unmap what we mapped
        self.kpt0()[2].store(0, Ordering::Relaxed);

        // Make sure we flush the TLBs
        fence(Ordering::SeqCst);
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
}

// SAFETY: All public operations are thread-safe
unsafe impl Send for Uat {}
unsafe impl Sync for Uat {}
