// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi GEM object implementation

use kernel::{
    bindings, c_str, drm,
    drm::{device, drv, gem, gem::shmem},
    error::{to_result, Result},
    io_mem::IoMem,
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::rtkit,
    sync::smutex::Mutex,
    sync::{Arc, ArcBorrow},
};

use kernel::drm::gem::BaseObject;

use crate::driver::AsahiDevice;

pub(crate) struct DriverObject {
    kernel: bool,
    flags: u32,
    mapping: Mutex<Option<crate::mmu::Mapping>>,
}

pub(crate) type Object = shmem::Object<DriverObject>;

pub(crate) struct ObjectRef {
    pub(crate) gem: gem::ObjectRef<shmem::Object<DriverObject>>,
    pub(crate) vmap: Option<shmem::VMap<DriverObject>>,
}

impl ObjectRef {
    pub(crate) fn vmap(&mut self) -> Result<&mut shmem::VMap<DriverObject>> {
        if self.vmap.is_none() {
            self.vmap = Some(self.gem.vmap()?);
        }
        Ok(self.vmap.as_mut().unwrap())
    }

    pub(crate) fn map_into(&mut self, vm: &crate::mmu::Vm) -> Result<usize> {
        let mut mapping = self.gem.mapping.lock();
        if mapping.is_some() {
            Err(EBUSY)
        } else {
            let sgt = self.gem.sg_table()?;
            let new_mapping = vm.map(self.gem.size(), &mut sgt.iter())?;

            let iova = new_mapping.iova();
            *mapping = Some(new_mapping);
            Ok(iova)
        }
    }

    pub(crate) fn map_into_range(
        &mut self,
        vm: &crate::mmu::Vm,
        start: u64,
        end: u64,
        alignment: u64,
        prot: u32,
    ) -> Result<usize> {
        let mut mapping = self.gem.p.mapping.lock();
        if mapping.is_some() {
            Err(EBUSY)
        } else {
            let sgt = self.gem.sg_table()?;
            let new_mapping = vm.map_in_range(
                self.gem.size(),
                &mut sgt.iter(),
                alignment,
                start,
                end,
                prot,
            )?;

            let iova = new_mapping.iova();
            *mapping = Some(new_mapping);
            Ok(iova)
        }
    }
}

pub(crate) fn new_kernel_object(dev: &AsahiDevice, size: usize) -> Result<ObjectRef> {
    let private = DriverObject {
        kernel: true,
        flags: 0,
        mapping: Mutex::new(None),
    };
    Ok(ObjectRef {
        gem: shmem::Object::new(dev, private, size)?,
        vmap: None,
    })
}

pub(crate) fn new_object(dev: &AsahiDevice, size: usize, flags: u32) -> Result<ObjectRef> {
    let private = DriverObject {
        kernel: false,
        flags,
        mapping: Mutex::new(None),
    };
    Ok(ObjectRef {
        gem: shmem::Object::new(dev, private, size)?,
        vmap: None,
    })
}

impl gem::BaseDriverObject<Object> for DriverObject {
    fn init(obj: &mut Object) -> Result<()> {
        dev_info!(obj.dev(), "DriverObject::init\n");
        Ok(())
    }
    fn uninit(obj: &mut Object) {
        dev_info!(obj.dev(), "DriverObject::uninit\n");
    }
}

impl shmem::DriverObject for DriverObject {
    type Driver = crate::driver::AsahiDriver;
}

impl rtkit::Buffer for ObjectRef {
    fn iova(&self) -> Option<usize> {
        Some(self.gem.p.mapping.lock().as_ref()?.iova())
    }
    fn buf(&mut self) -> Option<&mut [u8]> {
        let vmap = self.vmap.as_mut()?;
        Some(vmap.as_mut_slice())
    }
}
