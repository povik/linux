// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

use kernel::{
    c_str, device, drm, drm::drv, error::Result, io_mem::IoMem, of, platform, prelude::*, sync::Arc,
};

use crate::{gem, gpu, hw, mmu};

use kernel::macros::vtable;

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

pub(crate) struct AsahiData {
    dev: device::Device,
    gpu: Arc<dyn gpu::GpuManager>,
}

pub(crate) struct AsahiResources {
    asc: IoMem<ASC_CTL_SIZE>,
}

type DeviceData = device::Data<drv::Registration<AsahiDevice>, AsahiResources, AsahiData>;

pub(crate) struct AsahiDevice;

impl AsahiDevice {
    fn start_cpu(res: &mut AsahiResources) -> Result {
        let val = res.asc.readl_relaxed(CPU_CONTROL);

        res.asc.writel_relaxed(val | CPU_RUN, CPU_CONTROL);

        Ok(())
    }
}

#[vtable]
impl drv::Driver for AsahiDevice {
    type Data = ();
    type Object = gem::Object;

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

        pdev.set_dma_masks((1 << mmu::UAT_OAS) - 1)?;

        // TODO: add device abstraction to ioremap by name
        // SAFETY: AGX does DMA via the UAT IOMMU (mostly)
        let asc_res = unsafe { pdev.ioremap_resource(0)? };

        let mut res = AsahiResources {
            // SAFETY: This device does DMA via the UAT IOMMU.
            asc: asc_res,
        };

        // Start the coprocessor CPU, so UAT can initialize the handoff
        AsahiDevice::start_cpu(&mut res)?;

        let reg = drm::drv::Registration::<AsahiDevice>::new(&dev)?;
        let gpu = gpu::GpuManagerG13GV13_0B4::new(&reg.device(), &hw::t8103::HWCONFIG)?;

        let data =
            kernel::new_device_data!(reg, res, AsahiData { dev, gpu }, "Asahi::Registrations")?;

        let data = Arc::<DeviceData>::from(data);

        data.gpu.init()?;
        data.gpu.test()?;

        kernel::drm_device_register!(data.registrations().ok_or(ENXIO)?.as_pinned_mut(), (), 0)?;

        dev_info!(data.dev, "probed!\n");
        Ok(data)
    }
}
