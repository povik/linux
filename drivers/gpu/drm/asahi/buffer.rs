// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi ring buffer channels

use crate::debug::*;
use crate::fw::buffer;
use crate::fw::types::*;
use crate::util::*;
use crate::{alloc, fw, gpu, mmu, slotalloc, workqueue};
use crate::{box_in_place, place};
use core::cmp;
use core::sync::atomic::Ordering;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::{dbg, prelude::*};

const DEBUG_CLASS: DebugFlags = DebugFlags::Buffer;

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
    ualloc_priv: Arc<Mutex<alloc::SimpleAllocator>>,
    blocks: Vec<GpuArray<u8>>,
    max_blocks: usize,
    mgr: BufferManager,
    active_scenes: usize,
    active_slot: Option<slotalloc::Guard<SlotInner>>,
    last_token: Option<slotalloc::SlotToken>,
    tpc: Option<Arc<GpuArray<u8>>>,
    kernel_buffer: GpuArray<u8>,
    stats: GpuObject<buffer::Stats>,
    preempt1_size: usize,
    preempt2_size: usize,
    preempt3_size: usize,
    num_clusters: usize,
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
    preempt2_off: usize,
    preempt3_off: usize,
    meta2_off: usize,
    meta3_off: usize,
    meta4_off: usize,
}

pub(crate) struct TileInfo {
    pub(crate) tiles_x: u32,
    pub(crate) tiles_y: u32,
    pub(crate) tiles: u32,
    pub(crate) mtiles_x: u32,
    pub(crate) mtiles_y: u32,
    pub(crate) tiles_per_mtile_x: u32,
    pub(crate) tiles_per_mtile_y: u32,
    pub(crate) tiles_per_mtile: u32,
    pub(crate) tilemap_size: usize,
    pub(crate) tpc_size: usize,
    pub(crate) params: fw::vertex::raw::TilingParameters,
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

    pub(crate) fn tpc_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.tpc.gpu_pointer()
    }

    pub(crate) fn preempt_buf_1_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object.preempt_buf.gpu_pointer()
    }

    pub(crate) fn preempt_buf_2_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object
            .preempt_buf
            .gpu_offset_pointer(self.preempt2_off)
    }

    pub(crate) fn preempt_buf_3_pointer(&self) -> GpuPointer<'_, &'_ [u8]> {
        self.object
            .preempt_buf
            .gpu_offset_pointer(self.preempt3_off)
    }

    pub(crate) fn cluster_tilemaps_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.tilemaps.gpu_pointer())
    }

    pub(crate) fn meta_1_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_pointer())
    }

    pub(crate) fn meta_2_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_offset_pointer(self.meta2_off))
    }

    pub(crate) fn meta_3_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_offset_pointer(self.meta3_off))
    }

    pub(crate) fn meta_4_pointer(&self) -> Option<GpuPointer<'_, &'_ [u8]>> {
        self.object
            .clustering
            .as_ref()
            .map(|c| c.meta.gpu_offset_pointer(self.meta4_off))
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
        gpu: &dyn gpu::GpuManager,
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::SimpleAllocator>>,
        mgr: &BufferManager,
    ) -> Result<Buffer::ver> {
        let max_pages = MAX_SIZE / PAGE_SIZE;
        let max_blocks = MAX_SIZE / BLOCK_SIZE;
        let num_clusters = gpu.get_dyncfg().id.num_clusters as usize;
        let preempt1_size = num_clusters * gpu.get_cfg().preempt1_size;
        let preempt2_size = num_clusters * gpu.get_cfg().preempt2_size;
        let preempt3_size = num_clusters * gpu.get_cfg().preempt3_size;

        let inner = box_in_place!(buffer::Info::ver {
            block_ctl: alloc.shared.new_default::<buffer::BlockControl>()?,
            counter: alloc.shared.new_default::<buffer::Counter>()?,
            page_list: ualloc_priv.lock().array_empty(max_pages)?,
            block_list: ualloc_priv.lock().array_empty(max_blocks * 2)?,
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
                ualloc_priv,
                blocks: Vec::new(),
                max_blocks,
                mgr: mgr.clone(),
                active_scenes: 0,
                active_slot: None,
                last_token: None,
                tpc: None,
                kernel_buffer,
                stats,
                preempt1_size,
                preempt2_size,
                preempt3_size,
                num_clusters,
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
            inner.info.block_list[2 * (cur_count + i)] = page_num;
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
        tile_info: &TileInfo,
    ) -> Result<Scene::ver> {
        let mut inner = self.inner.lock();

        inner.stats.with(|raw, _inner| {
            raw.cpu_flag.store(1, Ordering::Relaxed);
        });

        let tilemap_size = tile_info.tilemap_size;
        let tpc_size = tile_info.tpc_size;

        // TODO: what is this exactly?
        mod_pr_debug!("Buffer: Allocating TVB buffers\n");
        let user_buffer = inner.ualloc.lock().array_empty(0x80)?;
        let tvb_heapmeta = inner.ualloc.lock().array_empty(0x200)?;
        let tvb_tilemap = inner.ualloc.lock().array_empty(tilemap_size)?;

        mod_pr_debug!("Buffer: Allocating misc buffers\n");
        let preempt_buf = inner
            .ualloc
            .lock()
            .array_empty(inner.preempt1_size + inner.preempt2_size + inner.preempt3_size)?;

        let mut seq_buf = inner.ualloc.lock().array_empty(0x800)?;
        for i in 1..0x400 {
            seq_buf[i] = (i + 1) as u64;
        }

        let tpc = match inner.tpc.as_ref() {
            Some(buf) if buf.len() >= tpc_size => buf.clone(),
            _ => {
                let buf = Arc::try_new(
                    inner
                        .ualloc_priv
                        .lock()
                        .array_empty((tpc_size + mmu::UAT_PGMSK) & !mmu::UAT_PGMSK)?,
                )?;
                inner.tpc = Some(buf.clone());
                buf
            }
        };

        let meta1_size = align(4 * inner.num_clusters, 0x80);
        // check
        let meta2_size = align(0x190 * inner.num_clusters, 0x80);
        let meta3_size = align(0x280 * inner.num_clusters, 0x80);
        let meta4_size = align(0x30 * inner.num_clusters, 0x80);
        let meta_size = meta1_size + meta2_size + meta3_size + meta4_size;

        let clustering = if inner.num_clusters > 1 {
            mod_pr_debug!("Buffer: Allocating clustering buffers\n");
            let tilemaps = inner
                .ualloc
                .lock()
                .array_empty(inner.num_clusters * tilemap_size)?;
            let meta = inner
                .ualloc
                .lock()
                .array_empty(inner.num_clusters * meta_size)?;
            Some(buffer::ClusterBuffers { tilemaps, meta })
        } else {
            None
        };

        let scene_inner = box_in_place!(buffer::Scene::ver {
            user_buffer: user_buffer,
            buffer: self.inner.clone(),
            tvb_heapmeta: tvb_heapmeta,
            tvb_tilemap: tvb_tilemap,
            tpc: tpc,
            clustering: clustering,
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

            mod_pr_debug!("Buffer: assigning slot {} (rebind={})", slot.slot(), rebind);

            inner.last_token = Some(slot.token());
            inner.active_slot = Some(slot);
        }

        inner.active_scenes += 1;

        Ok(Scene::ver {
            object: scene,
            slot: inner.active_slot.as_ref().unwrap().slot(),
            rebind,
            preempt2_off: inner.preempt1_size,
            preempt3_off: inner.preempt1_size + inner.preempt2_size,
            meta2_off: meta1_size,
            meta3_off: meta1_size + meta2_size,
            meta4_off: meta1_size + meta2_size + meta3_size,
        })
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
            mod_pr_debug!(
                "Buffer: no scenes left, dropping slot {}",
                inner.active_slot.take().unwrap().slot()
            );
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
