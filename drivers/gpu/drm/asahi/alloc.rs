// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi kernel object allocator

use kernel::{
    drm::{device, gem, gem::shmem, mm},
    error::Result,
    prelude::*,
};

use crate::object::{GpuArray, GpuObject, GpuStruct};

use alloc::fmt;
use core::mem;

pub(crate) trait Allocation<T> {
    fn ptr(&self) -> *mut T;
    fn gpu_ptr(&self) -> u64;
    fn size(&self) -> usize;

    fn device(&self) -> &device::Device;
}

pub(crate) trait Allocator {
    type Allocation<T>: Allocation<T>;

    fn new_object<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T) -> T::Raw<'a>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>>;

    fn new_boxed<T: GpuStruct>(
        &mut self,
        inner: Box<T>,
        callback: impl for<'a> FnOnce(&'a T) -> Result<Box<T::Raw<'a>>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>>;

    fn array_empty<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuArray<T, Self::Allocation<T>>>;

    fn device(&self) -> &device::Device;
}

pub(crate) struct SimpleAllocation<T> {
    dev: device::Device,
    ptr: *mut T,
    gpu_ptr: u64,
    size: usize,
    context: crate::mmu::Context,
    obj: crate::gem::ObjectRef,
}

impl<T> Allocation<T> for SimpleAllocation<T> {
    fn ptr(&self) -> *mut T {
        self.ptr
    }
    fn gpu_ptr(&self) -> u64 {
        self.gpu_ptr
    }
    fn size(&self) -> usize {
        self.size
    }

    fn device(&self) -> &device::Device {
        &self.dev
    }
}

pub(crate) struct SimpleAllocator {
    dev: device::Device,
    context: crate::mmu::Context,
    min_align: usize,
}

impl SimpleAllocator {
    pub(crate) fn new(
        dev: &device::Device,
        context: &crate::mmu::Context,
        min_align: usize,
    ) -> SimpleAllocator {
        SimpleAllocator {
            dev: dev.clone(),
            context: context.clone(),
            min_align,
        }
    }
}

impl Allocator for SimpleAllocator {
    type Allocation<T> = SimpleAllocation<T>;

    fn new<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T) -> T::Raw<'a>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>> {
        let size = mem::size_of::<T::Raw<'static>>();
        let size_aligned = (size + crate::mmu::UAT_PGSZ - 1) & !crate::mmu::UAT_PGMSK;
        let align = self.min_align.max(mem::align_of::<T::Raw<'static>>());
        let offset = (size_aligned - size) & !(align - 1);

        dev_info!(
            &self.dev,
            "Allocator::new: size={:#x} size_al={:#x} al={:#x} off={:#x}",
            size,
            size_aligned,
            align,
            offset
        );

        let mut obj = crate::gem::new_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        let map = obj.map_into(&self.context)?;

        let ptr = unsafe { p.add(offset) } as *mut T;
        let gpu_ptr = (map.iova() + offset) as u64;

        dev_info!(
            &self.dev,
            "Allocator::new -> {:#?} / {:#?} | {:#x} / {:#x}",
            p,
            ptr,
            map.iova(),
            gpu_ptr
        );

        let alloc = SimpleAllocation {
            dev: self.dev.clone(),
            ptr,
            gpu_ptr,
            size,
            context: self.context.clone(),
            obj,
        };

        GpuObject::<T, Self::Allocation<T>>::new(alloc, inner, callback)
    }

    fn new_boxed<T: GpuStruct>(
        &mut self,
        inner: Box<T>,
        callback: impl for<'a> FnOnce(&'a T) -> Result<Box<T::Raw<'a>>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>> {
        let size = mem::size_of::<T::Raw<'static>>();
        let size_aligned = (size + crate::mmu::UAT_PGSZ - 1) & !crate::mmu::UAT_PGMSK;
        let align = self.min_align.max(mem::align_of::<T::Raw<'static>>());
        let offset = (size_aligned - size) & !(align - 1);

        dev_info!(
            &self.dev,
            "Allocator::new: size={:#x} size_al={:#x} al={:#x} off={:#x}",
            size,
            size_aligned,
            align,
            offset
        );

        let mut obj = crate::gem::new_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        let map = obj.map_into(&self.context)?;

        let ptr = unsafe { p.add(offset) } as *mut T;
        let gpu_ptr = (map.iova() + offset) as u64;

        dev_info!(
            &self.dev,
            "Allocator::new -> {:#?} / {:#?} | {:#x} / {:#x}",
            p,
            ptr,
            map.iova(),
            gpu_ptr
        );

        let alloc = SimpleAllocation {
            dev: self.dev.clone(),
            ptr,
            gpu_ptr,
            size,
            context: self.context.clone(),
            obj,
        };

        GpuObject::<T, Self::Allocation<T>>::new_boxed(alloc, inner, callback)
    }

    fn array_empty<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuArray<T, Self::Allocation<T>>> {
        let size = mem::size_of::<T>() * count;
        let size_aligned = (size + crate::mmu::UAT_PGSZ - 1) & !crate::mmu::UAT_PGMSK;
        let align = self.min_align.max(mem::align_of::<T>());
        let offset = (size_aligned - size) & !(align - 1);

        dev_info!(
            &self.dev,
            "Allocator::array_empty: size={:#x} size_al={:#x} al={:#x} off={:#x} ({:#x} * {:#x})",
            size,
            size_aligned,
            align,
            offset,
            mem::size_of::<T>(),
            count
        );

        let mut obj = crate::gem::new_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        let ptr = unsafe { p.add(offset) } as *mut T;
        let map = obj.map_into(&self.context)?;
        let gpu_ptr = (map.iova() + offset) as u64;

        dev_info!(
            &self.dev,
            "Allocator::array_empty -> {:#?} / {:#?} | {:#x} / {:#x}",
            p,
            ptr,
            map.iova(),
            gpu_ptr
        );

        let alloc = SimpleAllocation {
            dev: self.dev.clone(),
            ptr,
            gpu_ptr,
            size,
            context: self.context.clone(),
            obj,
        };

        GpuArray::<T, Self::Allocation<T>>::empty(alloc, count)
    }

    fn device(&self) -> &device::Device {
        &self.dev
    }
}
