// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

use kernel::{
    c_str, device, drm, drm::drv, drm::ioctl, error::Result, io_mem::IoMem, of, platform,
    prelude::*, sync::Arc,
};

use crate::{file, gem, gpu, hw, mmu};

use kernel::macros::vtable;

const ASC_CTL_SIZE: usize = 0x4000;
const SGX_SIZE: usize = 0x1000000;
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
    pub(crate) dev: device::Device,
    pub(crate) gpu: Arc<dyn gpu::GpuManager>,
}

pub(crate) struct AsahiResources {
    asc: IoMem<ASC_CTL_SIZE>,
    sgx: IoMem<SGX_SIZE>,
}

type DeviceData = device::Data<drv::Registration<AsahiDriver>, AsahiResources, AsahiData>;

pub(crate) struct AsahiDriver;

pub(crate) type AsahiDevice = kernel::drm::device::Device<AsahiDriver>;

impl AsahiDriver {
    fn write32<const N: usize>(res: &mut IoMem<N>, off: usize, val: u32) {
        res.writel_relaxed(val, off);
    }

    fn init_mmio(res: &mut AsahiResources) -> Result {
        // Read: 0x0
        Self::write32(&mut res.sgx, 0xd14000, 0x70001);
        Ok(())
    }

    fn start_cpu(res: &mut AsahiResources) -> Result {
        let val = res.asc.readl_relaxed(CPU_CONTROL);

        res.asc.writel_relaxed(val | CPU_RUN, CPU_CONTROL);

        Ok(())
    }
}

#[vtable]
impl drv::Driver for AsahiDriver {
    type Data = Arc<DeviceData>;
    type File = file::File;
    type Object = gem::Object;

    const INFO: drv::DriverInfo = INFO;
    const FEATURES: u32 = drv::FEAT_GEM | drv::FEAT_RENDER;

    kernel::declare_drm_ioctls! {
        (ASAHI_SUBMIT,          drm_asahi_submit,
            ioctl::AUTH | ioctl::RENDER_ALLOW, file::File::submit),
        (ASAHI_WAIT_BO,         drm_asahi_wait_bo,
            ioctl::AUTH | ioctl::RENDER_ALLOW, file::File::wait_bo),
        (ASAHI_CREATE_BO,       drm_asahi_create_bo,
            ioctl::AUTH | ioctl::RENDER_ALLOW, file::File::create_bo),
        (ASAHI_MMAP_BO,         drm_asahi_mmap_bo,
            ioctl::AUTH | ioctl::RENDER_ALLOW, file::File::mmap_bo),
        (ASAHI_GET_PARAM,       drm_asahi_get_param,
                          ioctl::RENDER_ALLOW, file::File::get_param),
        (ASAHI_GET_BO_OFFSET,   drm_asahi_get_bo_offset,
            ioctl::AUTH | ioctl::RENDER_ALLOW, file::File::get_bo_offset),
    }
}

impl platform::Driver for AsahiDriver {
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
        let sgx_res = unsafe { pdev.ioremap_resource(1)? };

        let mut res = AsahiResources {
            // SAFETY: This device does DMA via the UAT IOMMU.
            asc: asc_res,
            sgx: sgx_res,
        };

        // Initialize misc MMIO
        AsahiDriver::init_mmio(&mut res)?;

        // Start the coprocessor CPU, so UAT can initialize the handoff
        AsahiDriver::start_cpu(&mut res)?;

        let reg = drm::drv::Registration::<AsahiDriver>::new(&dev)?;
        //let gpu = gpu::GpuManagerG13GV13_0B4::new(&reg.device(), &hw::t8103::HWCONFIG)?;
        let gpu = gpu::GpuManagerG13GV12_3::new(reg.device(), &hw::t8103::HWCONFIG)?;

        let data =
            kernel::new_device_data!(reg, res, AsahiData { dev, gpu }, "Asahi::Registrations")?;

        let data = Arc::<DeviceData>::from(data);

        data.gpu.init()?;
        data.gpu.test()?;

        kernel::drm_device_register!(
            data.registrations().ok_or(ENXIO)?.as_pinned_mut(),
            data.clone(),
            0
        )?;

        dev_info!(data.dev, "probed!\n");
        Ok(data)
    }
}
