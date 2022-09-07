// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

use kernel::{
    bindings, c_str, device, drm,
    drm::{drv, gem, gem::shmem},
    error::{to_result, Result},
    io_mem::IoMem,
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::rtkit,
    sync::smutex::Mutex,
    sync::{Arc, ArcBorrow},
    PointerWrapper,
};

use kernel::macros::vtable;

mod asahi_gem;
mod asahi_mmu;

const ASC_CTL_SIZE: usize = 0x4000;
const CPU_CONTROL: usize = 0x44;
const CPU_RUN: u32 = 0x1 << 4; // BIT(4)

const INFO: drv::DriverInfo = drv::DriverInfo {
    major: 0,
    minor: 0,
    patchlevel: 0,
    name: c_str!("asahi"),
    desc: c_str!("Apple AGX Graphics"),
    date: c_str!("20220831"),
};

struct AsahiData {
    dev: device::Device,
    uat: asahi_mmu::Uat,
    rtkit: Mutex<Option<rtkit::RTKit<AsahiDevice>>>,
}

struct AsahiResources {
    asc: IoMem<ASC_CTL_SIZE>,
}

type DeviceData = device::Data<drv::Registration<AsahiDevice>, AsahiResources, AsahiData>;

struct AsahiDevice;

impl AsahiDevice {
    fn start_cpu(res: &mut AsahiResources) -> Result {
        let val = res.asc.readl_relaxed(CPU_CONTROL);

        res.asc.writel_relaxed(val | CPU_RUN, CPU_CONTROL);

        Ok(())
    }
}

#[vtable]
impl rtkit::Operations for AsahiDevice {
    type Data = Arc<DeviceData>;
    type Buffer = asahi_gem::ObjectRef;

    fn shmem_alloc(
        data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        size: usize,
    ) -> Result<Self::Buffer> {
        let mut guard = data.registrations().ok_or(ENXIO)?;
        let mut reg = guard.as_pinned_mut();
        let dev = reg.device();
        dev_info!(dev, "shmem_alloc() {:#x} bytes\n", size);

        let mut obj = asahi_gem::new_object(dev, size)?;
        obj.vmap()?;
        let map = obj.map_into(data.uat.kernel_context())?;
        dev_info!(dev, "shmem_alloc() -> VA {:#x}\n", map.iova());
        Ok(obj)
    }
}

#[vtable]
impl drv::Driver for AsahiDevice {
    type Data = ();
    type Object = asahi_gem::Object;

    const INFO: drv::DriverInfo = INFO;
    const FEATURES: u32 = drv::FEAT_GEM | drv::FEAT_RENDER;
}

impl platform::Driver for AsahiDevice {
    type Data = Arc<DeviceData>;

    kernel::define_of_id_table! {(), [
        (of::DeviceId::Compatible(b"apple,agx-t8103"), None),
    ]}

    fn probe(
        pdev: &mut platform::Device,
        _id_info: Option<&Self::IdInfo>,
    ) -> Result<Arc<DeviceData>> {
        let dev = device::Device::from_dev(pdev);

        dev_info!(dev, "Probing!\n");

        pdev.set_dma_masks((1 << asahi_mmu::UAT_OAS) - 1);

        // TODO: add device abstraction to ioremap by name
        // SAFETY: AGX does DMA via the UAT IOMMU (mostly)
        let asc_res = unsafe { pdev.ioremap_resource(0)? };

        let mut res = AsahiResources {
            // SAFETY: This device does DMA via the UAT IOMMU.
            asc: asc_res,
        };

        // Start the coprocessor CPU, so UAT can initialize the handoff
        AsahiDevice::start_cpu(&mut res)?;

        let uat = asahi_mmu::Uat::new(&dev)?;
        let reg = drm::drv::Registration::<AsahiDevice>::new(&dev)?;

        let data = kernel::new_device_data!(
            reg,
            res,
            AsahiData {
                uat,
                dev,
                rtkit: Mutex::new(None),
            },
            "Asahi::Registrations"
        )?;

        let data = Arc::<DeviceData>::from(data);

        {
            let mut guard = data.registrations().ok_or(ENXIO)?;
            let mut reg = guard.as_pinned_mut();
            let mut dev = reg.device();
            dev_info!(dev, "info through dev\n");
        }

        let mut rtkit =
            unsafe { rtkit::RTKit::<AsahiDevice>::new(&data.dev, None, 0, data.clone()) }?;

        rtkit.boot()?;
        *data.rtkit.lock() = Some(rtkit);

        kernel::drm_device_register!(data.registrations().ok_or(ENXIO)?.as_pinned_mut(), (), 0)?;

        dev_info!(data.dev, "probed!\n");
        Ok(data)
    }
}

module_platform_driver! {
    type: AsahiDevice,
    name: "asahi",
    license: "Dual MIT/GPL",
}
