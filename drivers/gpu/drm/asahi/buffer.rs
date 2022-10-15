// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi ring buffer channels

use crate::fw::buffer;
use crate::fw::types::*;
use crate::{alloc, gpu, mmu, slotalloc, workqueue};
use crate::{box_in_place, place};
use core::cmp;
use core::sync::atomic::Ordering;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::{dbg, prelude::*};

const NUM_BUFFERS: u32 = 127;

pub(crate) const PAGE_SHIFT: usize = 15; // Buffer pages are 32K (!)
pub(crate) const PAGE_SIZE: usize = 1 << PAGE_SHIFT;
const PAGES_PER_BLOCK: usize = 4;
const BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK;

const MAX_SIZE: usize = 1 << 30; // 1 GiB

#[versions(AGX)]
pub(crate) struct BufferInner {
    info: GpuObject<buffer::Info::ver>,
    ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
    blocks: Vec<GpuArray<u8>>,
    max_blocks: usize,
    mgr: BufferManager,
    active_scenes: usize,
    active_slot: Option<slotalloc::Guard<SlotInner>>,
    last_token: Option<slotalloc::SlotToken>,
    tvb_something: GpuArray<u8>,
    kernel_buffer: GpuArray<u8>,
    stats: GpuObject<buffer::Stats>,
}

#[versions(AGX)]
pub(crate) struct Buffer {
    inner: Arc<Mutex<BufferInner::ver>>,
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct Scene {
    object: GpuObject<buffer::Scene::ver>,
    slot: u32,
    rebind: bool,
}

#[versions(AGX)]
impl Scene::ver {
    pub(crate) fn rebind(&self) -> bool {
        self.rebind
    }

    pub(crate) fn slot(&self) -> u32 {
        self.slot
    }

    pub(crate) fn gpu_pointer(&self) -> GpuPointer<'_, buffer::Scene::ver> {
        self.object.gpu_pointer()
    }

    pub(crate) fn weak_pointer(&self) -> GpuWeakPointer<buffer::Scene::ver> {
        self.object.weak_pointer()
    }

    pub(crate) fn kernel_buffer_pointer(&self) -> GpuWeakPointer<[u8]> {
        self.object.buffer.lock().kernel_buffer.weak_pointer()
    }

    pub(crate) fn buffer_pointer(&self) -> GpuWeakPointer<buffer::Info::ver> {
        self.object.buffer.lock().info.weak_pointer()
    }

    pub(crate) fn tvb_heapmeta_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.tvb_heapmeta.gpu_pointer()
    }

    pub(crate) fn tvb_tilemap_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.tvb_tilemap.gpu_pointer()
    }

    pub(crate) fn preempt_buf_pointer(&self) -> GpuPointer<'_, buffer::PreemptBuffer> {
        self.object.preempt_buf.gpu_pointer()
    }

    pub(crate) fn seq_buf_pointer(&self) -> GpuPointer<'_, &'_ [u64]> {
        self.object.seq_buf.gpu_pointer()
    }

    pub(crate) fn debug(&self) {
        dbg!(self);
        dbg!(&self.object.user_buffer);
    }
}

pub(crate) struct SlotInner();

impl slotalloc::SlotItem for SlotInner {
    type Owner = ();
}

pub(crate) struct BufferManager(slotalloc::SlotAllocator<SlotInner>);

#[versions(AGX)]
impl Buffer::ver {
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::SimpleAllocator>>,
        mgr: &BufferManager,
    ) -> Result<Buffer::ver> {
        let max_pages = MAX_SIZE / PAGE_SIZE;
        let max_blocks = MAX_SIZE / BLOCK_SIZE;

        let inner = box_in_place!(buffer::Info::ver {
            block_ctl: alloc.shared.new_default::<buffer::BlockControl>()?,
            counter: alloc.shared.new_default::<buffer::Counter>()?,
            page_list: ualloc_priv.lock().array_empty(max_pages)?,
            block_list: ualloc_priv.lock().array_empty(max_blocks)?,
        })?;

        let info = alloc.shared.new_boxed(inner, |inner, ptr| {
            Ok(place!(
                ptr,
                buffer::raw::Info::ver {
                    gpu_counter: 0x0,
                    unk_4: 0,
                    last_id: 0x0,
                    cur_id: -1,
                    unk_10: 0x0,
                    gpu_counter2: 0x0,
                    unk_18: 0x0,
                    #[ver(V < V13_0B4)]
                    unk_1c: 0x0,
                    page_list: inner.page_list.gpu_pointer(),
                    page_list_size: (4 * max_pages) as u32,
                    page_count: AtomicU32::new(0),
                    unk_30: 0xd1a,
                    block_count: AtomicU32::new(0),
                    unk_38: 0x0,
                    block_list: inner.block_list.gpu_pointer(),
                    block_ctl: inner.block_ctl.gpu_pointer(),
                    last_page: AtomicU32::new(0),
                    gpu_page_ptr1: 0x0,
                    gpu_page_ptr2: 0x0,
                    unk_58: 0x0,
                    block_size: BLOCK_SIZE as u32,
                    unk_60: U64(0x0),
                    counter: inner.counter.gpu_pointer(),
                    unk_70: 0x0,
                    unk_74: 0x0,
                    unk_78: 0x0,
                    unk_7c: 0x0,
                    unk_80: 0x1,
                    unk_84: 0x3468,
                    unk_88: 0x1178,
                    unk_8c: 0x0,
                    unk_90: Default::default(),
                }
            ))
        })?;

        let tvb_something = ualloc.lock().array_empty(0x20000)?;
        let kernel_buffer = alloc.private.array_empty(0x40)?;
        let stats = alloc
            .shared
            .new_object(Default::default(), |_inner| buffer::raw::Stats {
                cpu_flag: AtomicU32::from(1),
                ..Default::default()
            })?;

        Ok(Buffer::ver {
            inner: Arc::try_new(Mutex::new(BufferInner::ver {
                info,
                ualloc,
                blocks: Vec::new(),
                max_blocks,
                mgr: mgr.clone(),
                active_scenes: 0,
                active_slot: None,
                last_token: None,
                tvb_something,
                kernel_buffer,
                stats,
            }))?,
        })
    }

    pub(crate) fn block_count(&self) -> u32 {
        self.inner.lock().blocks.len() as u32
    }

    pub(crate) fn add_blocks(&mut self, count: usize) -> Result {
        let mut inner = self.inner.lock();

        let cur_count = inner.blocks.len();
        let new_count = cur_count + count;

        if new_count > inner.max_blocks {
            return Err(ENOMEM);
        }

        let mut new_blocks: Vec<GpuArray<u8>> = Vec::new();

        // Allocate the new blocks first, so if it fails they will be dropped
        let mut ualloc = inner.ualloc.lock();
        for _i in 0..count {
            new_blocks.try_push(ualloc.array_empty(BLOCK_SIZE)?)?;
        }
        core::mem::drop(ualloc);

        // Then actually commit them
        inner.blocks.try_reserve(count)?;

        for (i, block) in new_blocks.into_iter().enumerate() {
            let page_num = (block.gpu_va().get() >> PAGE_SHIFT) as u32;

            inner
                .blocks
                .try_push(block)
                .expect("try_push() failed after try_reserve()");
            inner.info.block_list[cur_count + i] = page_num;
            for j in 0..PAGES_PER_BLOCK {
                inner.info.page_list[(cur_count + i) * PAGES_PER_BLOCK + j] = page_num + j as u32;
            }
        }

        inner.info.block_ctl.with(|raw, _inner| {
            raw.total.store(new_count as u32, Ordering::SeqCst);
            raw.wptr.store(new_count as u32, Ordering::SeqCst);
        });

        let page_count = (new_count * PAGES_PER_BLOCK) as u32;
        inner.info.with(|raw, _inner| {
            raw.page_count.store(page_count, Ordering::Relaxed);
            raw.block_count.store(new_count as u32, Ordering::Relaxed);
            raw.last_page
                .store((page_count - 1) as u32, Ordering::Relaxed);
        });

        Ok(())
    }

    pub(crate) fn new_scene(
        &self,
        alloc: &mut gpu::KernelAllocators,
        tile_blocks: u32,
    ) -> Result<Scene::ver> {
        let mut inner = self.inner.lock();

        inner.stats.with(|raw, _inner| {
            raw.cpu_flag.store(1, Ordering::Relaxed);
        });

        // TODO: what is this exactly?
        let user_buffer = inner.ualloc.lock().array_empty(0x80)?;
        let tvb_heapmeta = inner.ualloc.lock().array_empty(0x200)?;
        let tvb_tilemap = inner
            .ualloc
            .lock()
            .array_empty(0x800 * tile_blocks as usize)?;
        let preempt_buf = inner.ualloc.lock().new_default::<buffer::PreemptBuffer>()?;
        let mut seq_buf = inner.ualloc.lock().array_empty(0x800)?;
        for i in 1..0x400 {
            seq_buf[i] = (i + 1) as u64;
        }
        let scene_inner = box_in_place!(buffer::Scene::ver {
            user_buffer: user_buffer,
            buffer: self.inner.clone(),
            tvb_heapmeta: tvb_heapmeta,
            tvb_tilemap: tvb_tilemap,
            preempt_buf: preempt_buf,
            seq_buf: seq_buf,
        })?;

        let stats_pointer = inner.stats.weak_pointer();
        let scene = alloc.private.new_boxed(scene_inner, |inner, ptr| {
            Ok(place!(
                ptr,
                buffer::raw::Scene {
                    unk_0: U64(0),
                    unk_8: U64(0),
                    unk_10: U64(0),
                    user_buffer: inner.user_buffer.gpu_pointer(),
                    unk_20: 0,
                    stats: stats_pointer,
                    unk_2c: 0,
                    unk_30: U64(0),
                    unk_38: U64(0),
                }
            ))
        })?;

        let mut rebind = false;

        if inner.active_slot.is_none() {
            assert_eq!(inner.active_scenes, 0);

            let slot = inner.mgr.0.get(inner.last_token)?;
            rebind = slot.changed();
            inner.last_token = Some(slot.token());
            inner.active_slot = Some(slot);
        }

        inner.active_scenes += 1;

        Ok(Scene::ver {
            object: scene,
            slot: inner.active_slot.as_ref().unwrap().slot(),
            rebind,
        })
    }

    pub(crate) fn tvb_something_pointer(&self) -> GpuWeakPointer<[u8]> {
        self.inner.lock().tvb_something.weak_pointer()
    }

    pub(crate) fn info_pointer(&self) -> GpuWeakPointer<buffer::Info::ver> {
        self.inner.lock().info.weak_pointer()
    }

    pub(crate) fn increment(&self) {
        let inner = self.inner.lock();
        inner.info.counter.with(|raw, _inner| {
            raw.count.fetch_add(1, Ordering::Relaxed);
        });
    }
}

#[versions(AGX)]
impl Clone for Buffer::ver {
    fn clone(&self) -> Self {
        Buffer::ver {
            inner: self.inner.clone(),
        }
    }
}

#[versions(AGX)]
impl Drop for Scene::ver {
    fn drop(&mut self) {
        let mut inner = self.object.buffer.lock();
        assert_ne!(inner.active_scenes, 0);
        inner.active_scenes -= 1;

        if inner.active_scenes == 0 {
            inner.active_slot = None;
        }
    }
}

impl BufferManager {
    pub(crate) fn new() -> Result<BufferManager> {
        Ok(BufferManager(slotalloc::SlotAllocator::new(
            NUM_BUFFERS,
            (),
            |_inner, _slot| SlotInner(),
        )?))
    }
}

impl Clone for BufferManager {
    fn clone(&self) -> Self {
        BufferManager(self.0.clone())
    }
}
