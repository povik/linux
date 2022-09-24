// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(clippy::unusual_byte_groupings)]

//! Asahi File state

use crate::driver::AsahiDevice;
use crate::{alloc, buffer, driver, gem, gpu, mmu, render};
use kernel::drm::gem::BaseObject;
use kernel::prelude::*;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::{bindings, drm};

pub(crate) struct File {
    vm: mmu::Vm,
    ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
    renderer: Box<dyn render::Renderer>,
}

type DrmFile = drm::file::File<File>;

impl drm::file::DriverFile for File {
    type Driver = driver::AsahiDriver;

    fn open(device: &AsahiDevice) -> Result<Box<Self>> {
        dev_info!(device, "DRM device opened");

        let vm = device.data().gpu.new_vm()?;
        let ualloc = Arc::try_new(Mutex::new(alloc::SimpleAllocator::new_with_range(
            device,
            &vm,
            0x60_00000000,
            0x60_ffffffff,
            mmu::PROT_GPU_FW_SHARED_RW,
            buffer::PAGE_SIZE,
        )))?;
        let renderer = device.data().gpu.new_renderer(ualloc.clone())?;

        Ok(Box::try_new(Self {
            vm,
            ualloc,
            renderer,
        })?)
    }
}

impl File {
    pub(crate) fn submit(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_submit,
        file: &DrmFile,
    ) -> Result<u32> {
        dev_info!(device, "IOCTL: submit");
        let inner = file.inner();
        inner
            .renderer
            .render(device, &inner.vm, &inner.ualloc, data)?;
        Ok(0)
    }

    pub(crate) fn wait_bo(
        device: &AsahiDevice,
        _data: &mut bindings::drm_asahi_wait_bo,
        _file: &DrmFile,
    ) -> Result<u32> {
        dev_info!(device, "IOCTL: wait_bo");
        Ok(0)
    }

    pub(crate) fn create_bo(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_create_bo,
        file: &DrmFile,
    ) -> Result<u32> {
        dev_info!(device, "IOCTL: create_bo");

        let mut bo = gem::new_object(device, data.size as usize, data.flags)?;

        if data.flags & bindings::ASAHI_BO_PIPELINE != 0 {
            let iova = bo.map_into_range(
                &file.inner().vm,
                0x11_00000000,
                0x11_ffffffff,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
            )?;
            data.offset = iova as u64 - 0x11_00000000;
        } else {
            let iova = bo.map_into_range(
                &file.inner().vm,
                0x15_00000000,
                0x1f_ffffffff,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
            )?;
            data.offset = iova as u64;
        }

        let handle = bo.gem.create_handle(file)?;
        data.handle = handle;

        Ok(0)
    }

    pub(crate) fn mmap_bo(
        device: &AsahiDevice,
        data: &mut bindings::drm_asahi_mmap_bo,
        file: &DrmFile,
    ) -> Result<u32> {
        dev_info!(device, "IOCTL: mmap_bo");

        let bo = gem::Object::lookup_handle(file, data.handle)?;

        data.offset = bo.create_mmap_offset()?;
        Ok(0)
    }

    pub(crate) fn get_param(
        device: &AsahiDevice,
        _data: &mut bindings::drm_asahi_get_param,
        _file: &DrmFile,
    ) -> Result<u32> {
        dev_info!(device, "IOCTL: get_param");
        Ok(0)
    }
    pub(crate) fn get_bo_offset(
        device: &AsahiDevice,
        _data: &mut bindings::drm_asahi_get_bo_offset,
        _file: &DrmFile,
    ) -> Result<u32> {
        dev_info!(device, "IOCTL: get_bo_offset");
        Ok(0)
    }
}
