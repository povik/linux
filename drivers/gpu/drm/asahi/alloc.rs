// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi kernel object allocator

use kernel::{
    drm::{device, gem, gem::shmem, mm},
    error::Result,
    prelude::*,
    str::CString,
    sync::Arc,
};

use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::mmu;
use crate::object::{GpuArray, GpuObject, GpuOnlyArray, GpuStruct, GpuWeakPointer};

use alloc::fmt;
use core::cell::UnsafeCell;
use core::cmp::Ordering;
use core::fmt::{Debug, Formatter};
use core::marker::PhantomData;
use core::mem;
use core::mem::MaybeUninit;
use core::ptr::NonNull;

const DEBUG_CLASS: DebugFlags = DebugFlags::Alloc;

#[cfg(not(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR))]
pub(crate) type DefaultAllocator = HeapAllocator;
#[cfg(not(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR))]
pub(crate) type DefaultAllocation = HeapAllocation;

#[cfg(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR)]
pub(crate) type DefaultAllocator = SimpleAllocator;
#[cfg(CONFIG_DRM_ASAHI_DEBUG_ALLOCATOR)]
pub(crate) type DefaultAllocation = SimpleAllocation;

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
        mod_dev_dbg!(
            self.device(),
            "SimpleAllocator: drop object @ {:#x}",
            self.gpu_ptr()
        );
        if debug_enabled(DebugFlags::FillAllocations) {
            if let Ok(vmap) = self.obj.vmap() {
                vmap.as_mut_slice().fill(0x42);
            }
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
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: &mmu::Vm,
        start: u64,
        end: u64,
        min_align: usize,
        prot: u32,
        _block_size: usize,
        _cpu_maps: bool,
        _name: fmt::Arguments<'_>,
    ) -> Result<SimpleAllocator> {
        Ok(SimpleAllocator {
            dev: dev.clone(),
            vm: vm.clone(),
            start,
            end,
            prot,
            min_align,
        })
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

        mod_dev_dbg!(
            &self.dev,
            "SimpleAllocator::new: size={:#x} size_al={:#x} al={:#x} off={:#x}",
            size,
            size_aligned,
            align,
            offset
        );

        let mut obj = crate::gem::new_kernel_object(&self.dev, size_aligned)?;
        let p = obj.vmap()?.as_mut_ptr() as *mut u8;
        if debug_enabled(DebugFlags::FillAllocations) {
            obj.vmap()?.as_mut_slice().fill(0xde);
        }
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

        mod_dev_dbg!(
            &self.dev,
            "SimpleAllocator::new -> {:#?} / {:#?} | {:#x} / {:#x}",
            p,
            ptr,
            iova,
            gpu_ptr
        );

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

pub(crate) struct HeapAllocationInner {
    dev: AsahiDevice,
    ptr: Option<NonNull<u8>>,
    real_size: usize,
    backing_objects: Arc<UnsafeCell<Vec<(crate::gem::ObjectRef, u64)>>>,
}

pub(crate) struct HeapAllocation(mm::Node<HeapAllocationInner>);

impl HeapAllocation {}

impl Drop for HeapAllocation {
    fn drop(&mut self) {
        mod_dev_dbg!(
            self.device(),
            "HeapAllocator: drop object @ {:#x}",
            self.gpu_ptr()
        );
    }
}

impl RawAllocation for HeapAllocation {
    // SAFETY: This function must always return a valid pointer.
    // Since the HeapAllocation contains a reference to the
    // backing_objects array that contains the object backing this pointer,
    // and objects are only ever added to it, this pointer is guaranteed to
    // remain valid for the lifetime of the HeapAllocation.
    fn ptr(&self) -> Option<NonNull<u8>> {
        self.0.ptr
    }
    // SAFETY: This function must always return a valid GPU pointer.
    // See the explanation in ptr().
    fn gpu_ptr(&self) -> u64 {
        self.0.start()
    }
    fn size(&self) -> usize {
        self.0.size() as usize
    }
    fn device(&self) -> &AsahiDevice {
        &self.0.dev
    }
}

pub(crate) struct HeapAllocator {
    dev: AsahiDevice,
    start: u64,
    end: u64,
    top: u64,
    prot: u32,
    vm: mmu::Vm,
    min_align: usize,
    block_size: usize,
    cpu_maps: bool,
    backing_objects: Arc<UnsafeCell<Vec<(crate::gem::ObjectRef, u64)>>>,
    guard_nodes: Vec<mm::Node<HeapAllocationInner>>,
    mm: mm::Allocator<HeapAllocationInner>,
    name: CString,
}

impl HeapAllocator {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        dev: &AsahiDevice,
        vm: &mmu::Vm,
        start: u64,
        end: u64,
        min_align: usize,
        prot: u32,
        block_size: usize,
        cpu_maps: bool,
        name: fmt::Arguments<'_>,
    ) -> Result<HeapAllocator> {
        if !min_align.is_power_of_two() {
            return Err(EINVAL);
        }

        let name = CString::try_from_fmt(name)?;

        let mm = mm::Allocator::new(start as u64, (end - start + 1) as u64)?;

        Ok(HeapAllocator {
            dev: dev.clone(),
            vm: vm.clone(),
            start,
            end,
            top: start,
            prot,
            min_align,
            block_size: block_size.max(min_align),
            cpu_maps,
            backing_objects: Arc::try_new(UnsafeCell::new(Vec::new()))?,
            guard_nodes: Vec::new(),
            mm,
            name,
        })
    }

    fn add_block(&mut self, size: usize) -> Result {
        let size_aligned = (size + mmu::UAT_PGSZ - 1) & !mmu::UAT_PGMSK;

        mod_dev_dbg!(
            &self.dev,
            "HeapAllocator[{}]::add_block: size={:#x} size_al={:#x}",
            &*self.name,
            size,
            size_aligned,
        );

        if self.top.saturating_add(size_aligned as u64) >= self.end {
            dev_err!(
                &self.dev,
                "HeapAllocator[{}]::add_block: Exhausted VA space",
                &*self.name,
            );
        }

        let mut obj = crate::gem::new_kernel_object(&self.dev, size_aligned)?;
        if self.cpu_maps && debug_enabled(DebugFlags::FillAllocations) {
            obj.vmap()?.as_mut_slice().fill(0xde);
        }

        let gpu_ptr = self.top;
        if let Err(e) = obj.map_at(&self.vm, gpu_ptr, self.prot, self.cpu_maps) {
            dev_err!(
                &self.dev,
                "HeapAllocator[{}]::add_block: Failed to map at {:#x} ({:?})",
                &*self.name,
                gpu_ptr,
                e
            );
            return Err(e);
        }

        self.backing_objects().try_reserve(1)?;

        let mut new_top = self.top + size_aligned as u64;
        if self.cpu_maps {
            let guard = self.min_align.max(mmu::UAT_PGSZ);
            mod_dev_dbg!(
                &self.dev,
                "HeapAllocator[{}]::add_block: Adding guard node {:#x}:{:#x}",
                &*self.name,
                new_top,
                guard
            );

            let inner = HeapAllocationInner {
                dev: self.dev.clone(),
                ptr: None,
                real_size: guard,
                backing_objects: self.backing_objects.clone(),
            };

            let node = match self.mm.reserve_node(inner, new_top, guard as u64, 0) {
                Ok(a) => a,
                Err(a) => {
                    dev_err!(
                        &self.dev,
                        "HeapAllocator[{}]::add_block: Failed to reserve guard node {:#x}:{:#x}: {:?}",
                        &*self.name,
                        guard,
                        new_top,
                        a
                    );
                    return Err(EIO);
                }
            };

            self.guard_nodes.try_push(node)?;

            new_top += guard as u64;
        }
        mod_dev_dbg!(
            &self.dev,
            "HeapAllocator[{}]::add_block: top={:#x}",
            &*self.name,
            new_top
        );

        self.backing_objects().try_push((obj, gpu_ptr))?;
        self.top = new_top;

        cls_dev_dbg!(
            MemStats,
            &self.dev,
            "{} Heap: grow to {} bytes",
            &*self.name,
            self.top - self.start
        );

        Ok(())
    }

    fn backing_objects(&mut self) -> &mut Vec<(crate::gem::ObjectRef, u64)> {
        // SAFETY:
        // This HeapAllocator is the only owner of a reference to the
        // backing_objects Arc that actually dereferences it or mutates
        // it in any way. Therefore, it is always safe to take a mutable
        // reference to it if we have a mutable reference to self.
        // The other references in HeapAlocations are only there to
        // keep the objects alive, and never interact with them.
        unsafe { &mut *self.backing_objects.get() }
    }

    fn find_obj(&mut self, addr: u64) -> Result<&mut (crate::gem::ObjectRef, u64)> {
        let idx = self
            .backing_objects()
            .binary_search_by(|obj| {
                let start = obj.1;
                let end = obj.1 + obj.0.size() as u64;
                if start > addr {
                    Ordering::Greater
                } else if end <= addr {
                    Ordering::Less
                } else {
                    Ordering::Equal
                }
            })
            .or(Err(ENOENT))?;

        Ok(&mut self.backing_objects()[idx])
    }
}

impl Allocator for HeapAllocator {
    type Raw = HeapAllocation;

    fn device(&self) -> &AsahiDevice {
        &self.dev
    }

    fn alloc(&mut self, size: usize, align: usize) -> Result<HeapAllocation> {
        if align != 0 && !align.is_power_of_two() {
            return Err(EINVAL);
        }
        let align = self.min_align.max(align);
        let size_aligned = (size + align - 1) & !(align - 1);

        mod_dev_dbg!(
            &self.dev,
            "HeapAllocator[{}]::new: size={:#x} size_al={:#x}",
            &*self.name,
            size,
            size_aligned,
        );

        let inner = HeapAllocationInner {
            dev: self.dev.clone(),
            ptr: None,
            real_size: size,
            // SAFETY: this keeps the backing objects alive even if the HeapAllocator is
            // destroyed, guaranteeing that `ptr` is valid if not-None.
            backing_objects: self.backing_objects.clone(),
        };

        let mut node = match self.mm.insert_node_generic(
            inner,
            size_aligned as u64,
            align as u64,
            0,
            mm::InsertMode::Best,
        ) {
            Ok(a) => a,
            Err(a) => {
                dev_err!(
                    &self.dev,
                    "HeapAllocator[{}]::new: Failed to insert node of size {:#x} / align {:#x}: {:?}",
                    &*self.name, size_aligned, align, a
                );
                return Err(a);
            }
        };

        let mut new_object = false;
        let start = node.start();
        let end = start + node.size();
        if end > self.top {
            if start > self.top {
                dev_warn!(
                    self.dev,
                    "HeapAllocator[{}]::alloc: top={:#x}, start={:#x}",
                    &*self.name,
                    self.top,
                    start
                );
            }
            let block_size = self.block_size.max((end - self.top) as usize);
            self.add_block(block_size)?;
            new_object = true;
        }
        assert!(end <= self.top);

        if self.cpu_maps {
            mod_dev_dbg!(
                self.dev,
                "HeapAllocator[{}]::alloc: mapping to CPU",
                &*self.name
            );

            let obj = if new_object {
                self.backing_objects().last_mut().unwrap()
            } else {
                match self.find_obj(start) {
                    Ok(a) => a,
                    Err(_) => {
                        dev_warn!(
                            self.dev,
                            "HeapAllocator[{}]::alloc: Failed to find object at {:#x}",
                            &*self.name,
                            start
                        );
                        return Err(EIO);
                    }
                }
            };
            assert!(obj.1 <= start);
            assert!(obj.1 + obj.0.size() as u64 >= end);
            let p = obj.0.vmap()?.as_mut_ptr() as *mut u8;
            node.ptr = NonNull::new(unsafe { p.add((start - obj.1) as usize) });
            mod_dev_dbg!(
                self.dev,
                "HeapAllocator[{}]::alloc: CPU pointer = {:?}",
                &*self.name,
                node.ptr
            );
        }

        mod_dev_dbg!(
            self.dev,
            "HeapAllocator[{}]::alloc: Allocated {:#x} bytes @ {:#x}",
            &*self.name,
            end - start,
            start
        );

        Ok(HeapAllocation(node))
    }
}
