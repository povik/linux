// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

use kernel::{
    c_str, device, drm, drm::drv, drm::ioctl, error::Result, of, platform, prelude::*, sync::Arc,
};

use crate::{debug, file, gem, gpu, hw, mmu, regs};

use kernel::macros::vtable;

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

type DeviceData = device::Data<drv::Registration<AsahiDriver>, regs::Resources, AsahiData>;

pub(crate) struct AsahiDriver;

pub(crate) type AsahiDevice = kernel::drm::device::Device<AsahiDriver>;

impl AsahiDriver {}

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
        (ASAHI_WAIT,            drm_asahi_wait,
            ioctl::AUTH | ioctl::RENDER_ALLOW, file::File::wait),
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
        debug::update_debug_flags();

        let dev = device::Device::from_dev(pdev);

        dev_info!(dev, "Probing!\n");

        pdev.set_dma_masks((1 << mmu::UAT_OAS) - 1)?;

        let res = regs::Resources::new(pdev)?;

        // Initialize misc MMIO
        res.init_mmio()?;

        // Start the coprocessor CPU, so UAT can initialize the handoff
        res.start_cpu()?;

        let reg = drm::drv::Registration::<AsahiDriver>::new(&dev)?;
        //let gpu = gpu::GpuManagerG13GV13_0B4::new(&reg.device(), &hw::t8103::HWCONFIG)?;
        let gpu = gpu::GpuManagerG13GV12_3::new(reg.device(), &hw::t8103::HWCONFIG)?;

        let data =
            kernel::new_device_data!(reg, res, AsahiData { dev, gpu }, "Asahi::Registrations")?;

        let data = Arc::<DeviceData>::from(data);

        data.gpu.init()?;

        kernel::drm_device_register!(
            data.registrations().ok_or(ENXIO)?.as_pinned_mut(),
            data.clone(),
            0
        )?;

        dev_info!(data.dev, "probed!\n");
        Ok(data)
    }
}
