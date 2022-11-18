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

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::mmu;
use crate::object::{GpuArray, GpuObject, GpuOnlyArray, GpuStruct, GpuWeakPointer};

use alloc::fmt;
use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;
use core::mem;
use core::mem::MaybeUninit;
use core::ptr::NonNull;

const DEBUG_CLASS: DebugFlags = DebugFlags::Alloc;

pub(crate) trait RawAllocation {
    fn ptr(&self) -> Option<NonNull<u8>>;
    fn gpu_ptr(&self) -> u64;
    fn size(&self) -> usize;

    fn device(&self) -> &AsahiDevice;
}

pub(crate) trait Allocation<T>: Debug {
    fn ptr(&self) -> Option<NonNull<T>>;
    fn gpu_ptr(&self) -> u64;
    fn size(&self) -> usize;

    fn device(&self) -> &AsahiDevice;
}

pub(crate) struct GenericAlloc<T, U: RawAllocation>(U, PhantomData<T>);

impl<T, U: RawAllocation> Allocation<T> for GenericAlloc<T, U> {
    fn ptr(&self) -> Option<NonNull<T>> {
        self.0
            .ptr()
            .map(|p| unsafe { NonNull::new_unchecked(p.as_ptr() as *mut T) })
    }
    fn gpu_ptr(&self) -> u64 {
        self.0.gpu_ptr()
    }
    fn size(&self) -> usize {
        self.0.size()
    }
    fn device(&self) -> &AsahiDevice {
        self.0.device()
    }
}

impl<T, U: RawAllocation> Debug for GenericAlloc<T, U> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct(core::any::type_name::<GenericAlloc<T, U>>())
            .field("ptr", &format_args!("{:?}", self.ptr()))
            .field("gpu_ptr", &format_args!("{:#X?}", self.gpu_ptr()))
            .field("size", &format_args!("{:#X?}", self.size()))
            .finish()
    }
}

pub(crate) trait Allocator {
    type Raw: RawAllocation;
    // TODO: Needs associated_type_defaults
    // type Allocation<T> = GenericAlloc<T, Self::Raw>;

    fn device(&self) -> &AsahiDevice;
    fn alloc(&mut self, size: usize, align: usize) -> Result<Self::Raw>;

    #[inline(never)]
    fn new_object<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T) -> T::Raw<'a>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new(self.alloc_object()?, inner, callback)
    }

    #[inline(never)]
    fn new_boxed<T: GpuStruct>(
        &mut self,
        inner: Box<T>,
        callback: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_boxed(self.alloc_object()?, inner, callback)
    }

    #[inline(never)]
    fn new_inplace<T: GpuStruct>(
        &mut self,
        inner: T,
        callback: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_inplace(
            self.alloc_object()?,
            inner,
            callback,
        )
    }

    #[inline(never)]
    fn new_default<T: GpuStruct + Default>(
        &mut self,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>>
    where
        for<'a> <T as GpuStruct>::Raw<'a>: Default,
    {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_default(self.alloc_object()?)
    }

    #[inline(never)]
    fn prealloc<T: GpuStruct>(&mut self) -> Result<GenericAlloc<T, Self::Raw>> {
        self.alloc_object()
    }

    #[inline(never)]
    fn new_prealloc<T: GpuStruct>(
        &mut self,
        inner_cb: impl FnOnce(GpuWeakPointer<T>) -> Result<Box<T>>,
        raw_cb: impl for<'a> FnOnce(&'a T, *mut MaybeUninit<T::Raw<'a>>) -> Result<&'a mut T::Raw<'a>>,
    ) -> Result<GpuObject<T, GenericAlloc<T, Self::Raw>>> {
        GpuObject::<T, GenericAlloc<T, Self::Raw>>::new_prealloc(
            self.alloc_object()?,
            inner_cb,
            raw_cb,
        )
    }

    fn alloc_object<T: GpuStruct>(&mut self) -> Result<GenericAlloc<T, Self::Raw>> {
        let size = mem::size_of::<T::Raw<'static>>();
        let align = mem::align_of::<T::Raw<'static>>();

        Ok(GenericAlloc(self.alloc(size, align)?, PhantomData))
    }

    fn array_empty<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuArray<T, GenericAlloc<T, Self::Raw>>> {
        let size = mem::size_of::<T>() * count;
        let align = mem::align_of::<T>();

        let alloc = GenericAlloc(self.alloc(size, align)?, PhantomData);
        GpuArray::<T, GenericAlloc<T, Self::Raw>>::empty(alloc, count)
    }

    fn array_gpuonly<T: Sized + Default>(
        &mut self,
        count: usize,
    ) -> Result<GpuOnlyArray<T, GenericAlloc<T, Self::Raw>>> {
        let size = mem::size_of::<T>() * count;
        let align = mem::align_of::<T>();

        let alloc = GenericAlloc(self.alloc(size, align)?, PhantomData);
        GpuOnlyArray::<T, GenericAlloc<T, Self::Raw>>::new(alloc, count)
    }
}

pub(crate) struct SimpleAllocation {
    dev: AsahiDevice,
    ptr: Option<NonNull<u8>>,
    gpu_ptr: u64,
    size: usize,
    vm: mmu::Vm,
    obj: crate::gem::ObjectRef,
}

impl Drop for SimpleAllocation {
    fn drop(&mut self) {
        /* dev_info!(
            self.device(),
            "Allocator: drop object @ {:#x}",
            self.gpu_ptr()
        ); */
        if let Ok(vmap) = self.obj.vmap() {
            vmap.as_mut_slice().fill(0x42);
        }
        self.obj.drop_mappings(self.vm.id());
    }
}

impl RawAllocation for SimpleAllocation {
    fn ptr(&self) -> Option<NonNull<u8>> {
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
}

impl Allocator for SimpleAllocator {
    type Raw = SimpleAllocation;

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }

    #[inline(never)]
    fn alloc(&mut self, size: usize, align: usize) -> Result<SimpleAllocation> {
        let size_aligned = (size + mmu::UAT_PGSZ - 1) & !mmu::UAT_PGMSK;
        let align = self.min_align.max(align);
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
        obj.vmap()?.as_mut_slice().fill(0xde);
        let iova = obj.map_into_range(
            &self.vm,
            self.start,
            self.end,
            self.min_align.max(mmu::UAT_PGSZ) as u64,
            self.prot,
            true,
        )?;

        let ptr = unsafe { p.add(offset) } as *mut u8;
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
            ptr: NonNull::new(ptr),
            gpu_ptr,
            size,
            vm: self.vm.clone(),
            obj,
        })
    }
}
