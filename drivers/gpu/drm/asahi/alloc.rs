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

use crate::driver::AsahiDevice;
use crate::mmu;
use crate::object::{GpuArray, GpuObject, GpuStruct, GpuWeakPointer};

use alloc::fmt;
use core::fmt::{Debug, Formatter};
use core::mem;
use core::mem::MaybeUninit;

pub(crate) trait Allocation<T>: Debug {
    fn ptr(&self) -> *mut T;
    fn gpu_ptr(&self) -> u64;
    fn size(&self) -> usize;

    fn device(&self) -> &AsahiDevice;
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
        callback: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>>;

    fn new_inplace<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>>;

    fn prealloc<T: GpuStruct>(&mut self) -> Result<Self::Allocation<T>>;

    fn new_prealloc<T: GpuStruct>(
        &mut self,
        inner_cb: impl FnOnce(GpuWeakPointer<T>) -> Result<Box<T>>,
        raw_cb: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>>;

    fn new_default<T: GpuStruct + Default>(&mut self) -> Result<GpuObject<T, Self::Allocation<T>>>
    where
        for<'a> <T as GpuStruct>::Raw<'a>: Default;

    fn array_empty<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuArray<T, Self::Allocation<T>>>;

    fn device(&self) -> &AsahiDevice;
}

pub(crate) struct SimpleAllocation<T> {
    dev: AsahiDevice,
    ptr: *mut T,
    gpu_ptr: u64,
    size: usize,
    vm: mmu::Vm,
    obj: crate::gem::ObjectRef,
}

impl<T> Debug for SimpleAllocation<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct(core::any::type_name::<SimpleAllocation<T>>())
            .field("ptr", &format_args!("{:p}", self.ptr))
            .field("gpu_ptr", &format_args!("{:#X?}", self.gpu_ptr))
            .field("size", &format_args!("{:#X?}", self.size))
            .finish()
    }
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

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }
}

pub(crate) struct SimpleAllocator {
    dev: AsahiDevice,
    start: u64,
    end: u64,
    prot: u32,
    vm: mmu::Vm,
    min_align: usize,
}

impl SimpleAllocator {
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: &mmu::Vm,
        min_align: usize,
        prot: u32,
    ) -> SimpleAllocator {
        SimpleAllocator {
            dev: dev.clone(),
            vm: vm.clone(),
            start: 0,
            end: u64::MAX,
            prot,
            min_align,
        }
    }

    pub(crate) fn new_with_range(
        dev: &AsahiDevice,
        vm: &mmu::Vm,
        start: u64,
        end: u64,
        prot: u32,
        min_align: usize,
    ) -> SimpleAllocator {
        SimpleAllocator {
            dev: dev.clone(),
            vm: vm.clone(),
            start,
            end,
            prot,
            min_align,
        }
    }

    #[inline(never)]
    fn alloc_object<T: GpuStruct>(&mut self) -> Result<SimpleAllocation<T>> {
        let size = mem::size_of::<T::Raw<'static>>();
        let size_aligned = (size + mmu::UAT_PGSZ - 1) & !mmu::UAT_PGMSK;
        let align = self.min_align.max(mem::align_of::<T::Raw<'static>>());
        let offset = (size_aligned - size) & !(align - 1);

        //         dev_info!(
        //             &self.dev,
        //             "Allocator::new: size={:#x} size_al={:#x} al={:#x} off={:#x}",
        //             size,
        //             size_aligned,
        //             align,
        //             offset
        //         );

        let mut obj = crate::gem::new_kernel_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        let iova = obj.map_into_range(
            &self.vm,
            self.start,
            self.end,
            self.min_align.max(mmu::UAT_PGSZ) as u64,
            self.prot,
        )?;

        let ptr = unsafe { p.add(offset) } as *mut T;
        let gpu_ptr = (iova + offset) as u64;

        //         dev_info!(
        //             &self.dev,
        //             "Allocator::new -> {:#?} / {:#?} | {:#x} / {:#x}",
        //             p,
        //             ptr,
        //             iova,
        //             gpu_ptr
        //         );

        Ok(SimpleAllocation {
            dev: self.dev.clone(),
            ptr,
            gpu_ptr,
            size,
            vm: self.vm.clone(),
            obj,
        })
    }
}

impl Allocator for SimpleAllocator {
    type Allocation<T> = SimpleAllocation<T>;

    #[inline(never)]
    fn new_object<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T) -> T::Raw<'a>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>> {
        GpuObject::<T, Self::Allocation<T>>::new(self.alloc_object()?, inner, callback)
    }

    #[inline(never)]
    fn new_boxed<T: GpuStruct>(
        &mut self,
        inner: Box<T>,
        callback: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>> {
        GpuObject::<T, Self::Allocation<T>>::new_boxed(self.alloc_object()?, inner, callback)
    }

    #[inline(never)]
    fn new_inplace<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>> {
        GpuObject::<T, Self::Allocation<T>>::new_inplace(self.alloc_object()?, inner, callback)
    }

    #[inline(never)]
    fn new_default<T: GpuStruct + Default>(&mut self) -> Result<GpuObject<T, Self::Allocation<T>>>
    where
        for<'a> <T as GpuStruct>::Raw<'a>: Default,
    {
        GpuObject::<T, Self::Allocation<T>>::new_default(self.alloc_object()?)
    }

    #[inline(never)]
    fn prealloc<T: GpuStruct>(&mut self) -> Result<Self::Allocation<T>> {
        self.alloc_object()
    }

    #[inline(never)]
    fn new_prealloc<T: GpuStruct>(
        &mut self,
        inner_cb: impl FnOnce(GpuWeakPointer<T>) -> Result<Box<T>>,
        raw_cb: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, Self::Allocation<T>>> {
        GpuObject::<T, Self::Allocation<T>>::new_prealloc(self.alloc_object()?, inner_cb, raw_cb)
    }

    fn array_empty<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuArray<T, Self::Allocation<T>>> {
        let size = mem::size_of::<T>() * count;
        let size_aligned = (size + mmu::UAT_PGSZ - 1) & !mmu::UAT_PGMSK;
        let align = self.min_align.max(mem::align_of::<T>());
        let offset = (size_aligned - size) & !(align - 1);

        //         dev_info!(
        //             &self.dev,
        //             "Allocator::array_empty: size={:#x} size_al={:#x} al={:#x} off={:#x} ({:#x} * {:#x})",
        //             size,
        //             size_aligned,
        //             align,
        //             offset,
        //             mem::size_of::<T>(),
        //             count
        //         );

        let mut obj = crate::gem::new_kernel_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        let ptr = unsafe { p.add(offset) } as *mut T;
        let iova = obj.map_into_range(
            &self.vm,
            self.start,
            self.end,
            self.min_align.max(mmu::UAT_PGSZ) as u64,
            self.prot,
        )?;
        let gpu_ptr = (iova + offset) as u64;

        //         dev_info!(
        //             &self.dev,
        //             "Allocator::array_empty -> {:#?} / {:#?} | {:#x} / {:#x}",
        //             p,
        //             ptr,
        //             iova,
        //             gpu_ptr
        //         );

        let alloc = SimpleAllocation {
            dev: self.dev.clone(),
            ptr,
            gpu_ptr,
            size,
            vm: self.vm.clone(),
            obj,
        };

        GpuArray::<T, Self::Allocation<T>>::empty(alloc, count)
    }

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }
}
