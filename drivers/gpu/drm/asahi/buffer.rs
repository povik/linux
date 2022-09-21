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

const PAGE_SHIFT: usize = mmu::UAT_PGBIT;
const PAGE_SIZE: usize = mmu::UAT_PGSZ;
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
}

#[versions(AGX)]
pub(crate) struct Buffer {
    inner: Arc<Mutex<BufferInner::ver>>,
}

#[versions(AGX)]
pub(crate) struct Scene(GpuObject<buffer::Scene::ver>);

pub(crate) struct SlotInner();

impl slotalloc::SlotItem for SlotInner {
    type Owner = ();
}

#[versions(AGX)]
impl Buffer::ver {
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
        mgr: &BufferManager,
    ) -> Result<Buffer::ver> {
        let max_pages = MAX_SIZE / PAGE_SIZE;
        let max_blocks = MAX_SIZE / BLOCK_SIZE;

        let inner = box_in_place!(buffer::Info::ver {
            block_ctl: alloc.shared.new_default::<buffer::BlockControl>()?,
            counter: alloc.shared.new_default::<buffer::Counter>()?,
            page_list: alloc.shared.array_empty(max_pages)?,
            block_list: alloc.shared.array_empty(max_blocks)?,
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

        Ok(Buffer::ver {
            inner: Arc::try_new(Mutex::new(BufferInner::ver {
                info,
                ualloc,
                blocks: Vec::new(),
                max_blocks,
                mgr: mgr.clone(),
            }))?,
        })
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
            inner.info.page_list[(cur_count + i) * PAGES_PER_BLOCK] = page_num;
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

    pub(crate) fn new_scene(&self, alloc: &mut gpu::KernelAllocators) -> Result<Scene::ver> {
        let inner = self.inner.lock();

        // TODO: what is this exactly?
        let user_buffer = inner.ualloc.lock().array_empty(0x80)?;
        let scene_inner = box_in_place!(buffer::Scene::ver {
            user_buffer: user_buffer,
            stats: alloc.shared.new_default::<buffer::Stats>()?,
            buffer: self.inner.clone(),
        })?;

        let scene = alloc.private.new_boxed(scene_inner, |inner, ptr| {
            Ok(place!(
                ptr,
                buffer::raw::Scene {
                    unk_0: U64(0),
                    unk_8: U64(0),
                    unk_10: U64(0),
                    user_buffer: inner.user_buffer.gpu_pointer(),
                    unk_20: 0,
                    stats: inner.stats.gpu_pointer(),
                    unk_2c: 0,
                    unk_30: U64(0),
                    unk_38: U64(0),
                }
            ))
        })?;

        Ok(Scene::ver(scene))
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

pub(crate) struct BufferManager(slotalloc::SlotAllocator<SlotInner>);

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
