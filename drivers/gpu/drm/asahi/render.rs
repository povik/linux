// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(clippy::unusual_byte_groupings)]

//! Asahi File state

use crate::alloc::Allocator;
use crate::debug::*;
use crate::driver::AsahiDevice;
use crate::fw::types::*;
use crate::gpu::GpuManager;
use crate::{alloc, buffer, channel, driver, event, fw, gem, gpu, microseq, mmu, workqueue};
use crate::{box_in_place, inner_ptr, inner_weak_ptr, place};
use core::mem::MaybeUninit;
use kernel::bindings;
use kernel::drm::gem::BaseObject;
use kernel::io_buffer::IoBufferReader;
use kernel::prelude::*;
use kernel::sync::{smutex::Mutex, Arc};
use kernel::user_ptr::UserSlicePtr;

const DEBUG_CLASS: DebugFlags = DebugFlags::Render;

pub(crate) trait Renderer: Send + Sync {
    fn render(
        &self,
        vm: &mmu::Vm,
        ualloc: &Arc<Mutex<alloc::SimpleAllocator>>,
        cmd: &bindings::drm_asahi_submit,
        id: u64,
    ) -> Result;
}

#[versions(AGX)]
pub(crate) struct Renderer {
    dev: AsahiDevice,
    wq_vtx: Arc<workqueue::WorkQueue>,
    wq_frag: Arc<workqueue::WorkQueue>,
    buffer: buffer::Buffer::ver,
    gpu_context: GpuObject<fw::workqueue::GpuContextData>,
    notifier_list: GpuObject<fw::event::NotifierList>,
    notifier: Arc<GpuObject<fw::event::Notifier>>,
    id: u64,
}

#[versions(AGX)]
unsafe impl Send for Renderer::ver {}
#[versions(AGX)]
unsafe impl Sync for Renderer::ver {}

struct TileInfo {
    tiles_x: u32,
    tiles_y: u32,
    tiles: u32,
    tile_blocks_x: u32,
    tile_blocks_y: u32,
    tile_blocks: u32,
    params: fw::vertex::raw::TilingParameters,
}

#[versions(AGX)]
impl Renderer::ver {
    pub(crate) fn new(
        dev: &AsahiDevice,
        alloc: &mut gpu::KernelAllocators,
        ualloc: Arc<Mutex<alloc::SimpleAllocator>>,
        ualloc_priv: Arc<Mutex<alloc::SimpleAllocator>>,
        event_manager: Arc<event::EventManager>,
        mgr: &buffer::BufferManager,
        id: u64,
    ) -> Result<Renderer::ver> {
        let mut buffer = buffer::Buffer::ver::new(alloc, ualloc, ualloc_priv, mgr)?;

        buffer.add_blocks(0x10)?;

        let gpu_context: GpuObject<fw::workqueue::GpuContextData> = alloc
            .shared
            .new_object(Default::default(), |_inner| Default::default())?;

        let mut notifier_list = alloc.private.new_default::<fw::event::NotifierList>()?;

        let self_ptr = notifier_list.weak_pointer();
        notifier_list.with_mut(|raw, _inner| {
            raw.list_head.next = Some(inner_weak_ptr!(self_ptr, list_head));
        });

        let notifier: Arc<GpuObject<fw::event::Notifier>> =
            Arc::try_new(alloc.private.new_inplace(
                fw::event::Notifier {
                    threshold: alloc.shared.new_default::<fw::event::Threshold>()?,
                },
                |inner, ptr: *mut MaybeUninit<fw::event::raw::Notifier<'_>>| {
                    Ok(place!(
                        ptr,
                        fw::event::raw::Notifier {
                            threshold: inner.threshold.gpu_pointer(),
                            generation: AtomicU32::new(id as u32),
                            cur_count: AtomicU32::new(0),
                            unk_10: AtomicU32::new(0x50),
                            state: Default::default()
                        }
                    ))
                },
            )?)?;

        Ok(Renderer::ver {
            dev: dev.clone(),
            wq_vtx: workqueue::WorkQueue::new(
                alloc,
                event_manager.clone(),
                gpu_context.weak_pointer(),
                notifier_list.weak_pointer(),
                channel::PipeType::Vertex,
                id,
            )?,
            wq_frag: workqueue::WorkQueue::new(
                alloc,
                event_manager,
                gpu_context.weak_pointer(),
                notifier_list.weak_pointer(),
                channel::PipeType::Fragment,
                id,
            )?,
            buffer,
            gpu_context,
            notifier_list,
            notifier,
            id,
        })
    }

    fn get_tiling_params(cmdbuf: &bindings::drm_asahi_cmdbuf) -> Result<TileInfo> {
        let width: u32 = cmdbuf.fb_width;
        let height: u32 = cmdbuf.fb_height;

        if width > 65536 || height > 65536 {
            return Err(EINVAL);
        }

        let tile_width = 32u32;
        let tile_height = 32u32;
        let tiles_x = (width + tile_width - 1) / tile_width;
        let tiles_y = (height + tile_height - 1) / tile_height;
        let tiles = tiles_x * tiles_y;

        let tile_blocks_x = (tiles_x + 15) / 16;
        let tile_blocks_y = (tiles_y + 15) / 16;
        let tile_blocks = tile_blocks_x * tile_blocks_y;

        Ok(TileInfo {
            tiles_x,
            tiles_y,
            tiles,
            tile_blocks_x,
            tile_blocks_y,
            tile_blocks,
            params: fw::vertex::raw::TilingParameters {
                size1: 0x14 * tile_blocks,
                ppp_multisamplectl: 0x88,
                ppp_ctrl: cmdbuf.ppp_ctrl,
                x_max: (width - 1) as u16,
                y_max: (height - 1) as u16,
                te_screen: ((tiles_y - 1) << 12) | (tiles_x - 1),
                te_mtile1: (12 * tile_blocks_x) | (tile_blocks_x << 12) | (tile_blocks_x << 20),
                te_mtile2: (12 * tile_blocks_y) | (tile_blocks_y << 12) | (tile_blocks_y << 20),
                size2: 0x10 * tile_blocks,
                size3: 0x20 * tile_blocks,
                unk_24: 0x100,
                unk_28: 0x8000,
            },
        })
    }
}

#[versions(AGX)]
impl Renderer for Renderer::ver {
    fn render(
        &self,
        vm: &mmu::Vm,
        ualloc: &Arc<Mutex<alloc::SimpleAllocator>>,
        cmd: &bindings::drm_asahi_submit,
        id: u64,
    ) -> Result {
        let dev = self.dev.data();
        let gpu = match dev.gpu.as_any().downcast_ref::<gpu::GpuManager::ver>() {
            Some(gpu) => gpu,
            None => panic!("GpuManager mismatched with Renderer!"),
        };
        let notifier = &self.notifier;

        self.buffer.increment();

        let mut alloc = gpu.alloc();
        let kalloc = &mut *alloc;

        mod_dev_dbg!(self.dev, "[Submission {}] Render!\n", id);

        let mut cmdbuf_reader = unsafe {
            UserSlicePtr::new(
                cmd.cmdbuf as usize as *mut _,
                core::mem::size_of::<bindings::drm_asahi_cmdbuf>(),
            )
            .reader()
        };

        let mut cmdbuf: MaybeUninit<bindings::drm_asahi_cmdbuf> = MaybeUninit::uninit();
        unsafe {
            cmdbuf_reader.read_raw(
                cmdbuf.as_mut_ptr() as *mut u8,
                core::mem::size_of::<bindings::drm_asahi_cmdbuf>(),
            )?;
        }
        let cmdbuf = unsafe { cmdbuf.assume_init() };

        if cmdbuf.fb_width == 0
            || cmdbuf.fb_height == 0
            || cmdbuf.fb_width > 16384
            || cmdbuf.fb_height > 16384
        {
            mod_dev_dbg!(
                self.dev,
                "[Submission {}] Invalid dimensions {}x{}\n",
                id,
                cmdbuf.fb_width,
                cmdbuf.fb_height
            );
            return Err(EINVAL);
        }

        // This sequence number increases per new client/VM? assigned to some slot,
        // but it's unclear *which* slot...
        let slot_client_seq: u8 = (self.id & 0xff) as u8;

        let tile_info = Self::get_tiling_params(&cmdbuf)?;

        let mut batches_vtx = workqueue::WorkQueue::begin_batch(&self.wq_vtx)?;
        let mut batches_frag = workqueue::WorkQueue::begin_batch(&self.wq_frag)?;

        let scene = Arc::try_new(self.buffer.new_scene(kalloc, tile_info.tile_blocks)?)?;

        let next_vtx = batches_vtx.event_value().next();
        let next_frag = batches_frag.event_value().next();
        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Vert event #{} {:#x?} -> {:#x?}\n",
            id,
            batches_vtx.event().slot(),
            batches_vtx.event_value(),
            next_vtx
        );
        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Frag event #{} {:#x?} -> {:#x?}\n",
            id,
            batches_frag.event().slot(),
            batches_frag.event_value(),
            next_frag
        );

        let vm_bind = gpu.bind_vm(vm)?;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] VM slot = {}\n",
            id,
            vm_bind.slot()
        );

        let uuid_3d = cmdbuf.cmd_3d_id;
        let uuid_ta = cmdbuf.cmd_ta_id;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Vert UUID = {:#x?}\n",
            id,
            uuid_ta
        );
        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Frag UUID = {:#x?}\n",
            id,
            uuid_3d
        );

        let barrier: GpuObject<fw::workqueue::Barrier> = kalloc.private.new_inplace(
            Default::default(),
            |_inner, ptr: *mut MaybeUninit<fw::workqueue::raw::Barrier>| {
                Ok(place!(
                    ptr,
                    fw::workqueue::raw::Barrier {
                        tag: fw::workqueue::CommandType::Barrier,
                        wait_stamp: batches_vtx.event().fw_stamp_pointer(),
                        wait_value: next_vtx,
                        wait_slot: batches_vtx.event().slot(),
                        stamp_self: next_frag,
                        uuid: uuid_3d,
                        unk: 0,
                    }
                ))
            },
        )?;

        batches_frag.add(Box::try_new(barrier)?)?;

        let unk0 = 0x0;
        let unk1 = 0x0;

        let frag = GpuObject::new_prealloc(
            kalloc.private.prealloc()?,
            |ptr: GpuWeakPointer<fw::fragment::RunFragment::ver>| {
                let mut builder = microseq::Builder::new();

                let stats = inner_weak_ptr!(
                    gpu.initdata.runtime_pointers.stats.frag.weak_pointer(),
                    stats
                );

                let mut attachments: Array<0x10, microseq::Attachment> = Default::default();
                let mut num_attachments = 0;

                for i in 0..cmdbuf.attachment_count.min(cmdbuf.attachments.len() as u32) {
                    let att = &cmdbuf.attachments[i as usize];
                    let cache_lines = (att.size + 127) >> 7;
                    let order = 1;
                    attachments[i as usize] = microseq::Attachment {
                        address: U64(att.pointer),
                        size: cache_lines,
                        unk_c: 0x17,
                        unk_e: order,
                    };
                    num_attachments += 1;
                }

                let start_frag = builder.add(microseq::StartFragment::ver {
                    header: microseq::op::StartFragment::HEADER,
                    job_params2: inner_weak_ptr!(ptr, job_params2),
                    job_params1: inner_weak_ptr!(ptr, job_params1),
                    scene: scene.gpu_pointer(),
                    stats,
                    busy_flag: inner_weak_ptr!(ptr, busy_flag),
                    tvb_overflow_count: inner_weak_ptr!(ptr, tvb_overflow_count),
                    unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                    work_queue: self.wq_frag.info_pointer(),
                    work_item: ptr,
                    vm_slot: vm_bind.slot(),
                    unk_50: 0x1, // fixed
                    event_generation: self.id as u32,
                    buffer_slot: scene.slot(),
                    unk_5c: 0,
                    prev_stamp_value: U64(batches_frag.event_value().counter() as u64),
                    unk_68: 0,
                    unk_758_flag: inner_weak_ptr!(ptr, unk_758_flag),
                    unk_job_buf: inner_weak_ptr!(ptr, meta.unk_buf_0),
                    unk_7c: 0,
                    unk_80: 0,
                    unk_84: 0,
                    uuid: uuid_3d,
                    attachments,
                    num_attachments,
                    unk_190: 0,
                    #[ver(V >= V13_0B4)]
                    unk_194: U64(0),
                    #[ver(V >= V13_0B4)]
                    notifier_buf: inner_weak_ptr!(&notifier.weak_pointer(), state.unk_buf),
                })?;

                builder.add(microseq::WaitForIdle {
                    header: microseq::op::WaitForIdle::new(microseq::Pipe::Fragment),
                })?;

                let off = builder.offset_to(start_frag);
                builder.add(microseq::FinalizeFragment::ver {
                    header: microseq::op::FinalizeFragment::HEADER,
                    uuid: uuid_3d,
                    unk_8: 0,
                    fw_stamp: batches_frag.event().fw_stamp_pointer(),
                    stamp_value: next_frag,
                    unk_18: 0,
                    scene: scene.weak_pointer(),
                    buffer: scene.buffer_pointer(),
                    unk_2c: U64(1),
                    stats,
                    unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                    busy_flag: inner_weak_ptr!(ptr, busy_flag),
                    work_queue: self.wq_frag.info_pointer(),
                    work_item: ptr,
                    vm_slot: vm_bind.slot(),
                    unk_60: 0,
                    unk_758_flag: inner_weak_ptr!(ptr, unk_758_flag),
                    unk_6c: U64(0),
                    unk_74: U64(0),
                    unk_7c: U64(0),
                    unk_84: U64(0),
                    unk_8c: U64(0),
                    restart_branch_offset: off,
                    unk_98: 0,
                    #[ver(V >= V13_0B4)]
                    unk_9c: Default::default(),
                })?;

                builder.add(microseq::RetireStamp {
                    header: microseq::op::RetireStamp::HEADER,
                })?;

                Ok(box_in_place!(fw::fragment::RunFragment::ver {
                    notifier: notifier.clone(),
                    scene: scene.clone(),
                    micro_seq: builder.build(&mut kalloc.private)?,
                    vm_bind: vm_bind.clone(),
                    aux_fb: ualloc.lock().array_empty(0x8000)?,
                })?)
            },
            |inner, ptr| {
                let aux_fb_info = fw::fragment::raw::AuxFBInfo::ver {
                    unk1: 0xc000,
                    unk2: 0,
                    width: cmdbuf.fb_width,
                    height: cmdbuf.fb_height,
                    #[ver(V >= V13_0B4)]
                    unk3: 0x100000,
                };

                let unk_flag = false;
                let aux_fb_unk: u32 = if unk_flag { 4 } else { 8 };
                let unk_378: u32 = aux_fb_unk * 2;

                Ok(place!(
                    ptr,
                    fw::fragment::raw::RunFragment::ver {
                        tag: fw::workqueue::CommandType::RunFragment,
                        #[ver(V >= V13_0B4)]
                        counter: 1,
                        vm_slot: vm_bind.slot(),
                        unk_8: 0,
                        microsequence: inner.micro_seq.gpu_pointer(),
                        microsequence_size: inner.micro_seq.len() as u32,
                        notifier: inner.notifier.gpu_pointer(),
                        buffer: inner.scene.buffer_pointer(),
                        scene: inner.scene.gpu_pointer(),
                        unk_buffer_buf: inner.scene.kernel_buffer_pointer(),
                        tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                        unk_40: U64(0x88),
                        unk_48: 0x1,
                        tile_blocks_y: 4 * tile_info.tile_blocks_y as u16,
                        tile_blocks_x: 4 * tile_info.tile_blocks_x as u16,
                        unk_50: U64(0),
                        unk_58: U64(0),
                        uuid1: 0x3b315cae, // ??,
                        uuid2: 0x3b6c7b92, // ??,
                        unk_68: U64(0),
                        tile_count: U64(tile_info.tiles as u64),
                        job_params1: fw::fragment::raw::JobParameters1::ver {
                            unk_0: U64(0xa000), // - maybe tvb_something_size?
                            clear_pipeline: fw::fragment::raw::ClearPipelineBinding {
                                pipeline_bind: U64(cmdbuf.load_pipeline_bind as u64),
                                address: U64(cmdbuf.load_pipeline as u64 | 4),
                            },
                            unk_18: U64(0x88),
                            scissor_array: U64(cmdbuf.scissor_array),
                            depth_bias_array: U64(cmdbuf.depth_bias_array),
                            aux_fb_info: aux_fb_info,
                            depth_dimensions: U64(cmdbuf.depth_dimensions as u64),
                            unk_48: U64(0x0),
                            zls_ctrl: U64(cmdbuf.zls_ctrl),
                            depth_buffer_ptr1: U64(cmdbuf.depth_buffer_1),
                            depth_buffer_ptr2: U64(cmdbuf.depth_buffer_2),
                            stencil_buffer_ptr1: U64(cmdbuf.stencil_buffer_1),
                            stencil_buffer_ptr2: U64(cmdbuf.stencil_buffer_2),
                            unk_68: Default::default(),
                            tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                            tvb_heapmeta: inner.scene.tvb_heapmeta_pointer(),
                            unk_e8: U64(0x50000000u64 * (tile_info.tile_blocks as u64)),
                            tvb_heapmeta_2: inner.scene.tvb_heapmeta_pointer(),
                            // 0x10000 - clear empty tiles
                            unk_f8: U64(0x10280), //#0x10280 # TODO: varies 0, 0x280, 0x10000, 0x10280
                            aux_fb: inner.aux_fb.gpu_pointer(),
                            unk_108: Default::default(),
                            pipeline_base: U64(0x11_00000000),
                            unk_140: U64(0x8c60),
                            unk_148: U64(0x0),
                            unk_150: U64(0x0),
                            unk_158: U64(0x1c),
                            unk_160_padding: Default::default(),
                            #[ver(V < V13_0B4)]
                            __pad0: Default::default(),
                        },
                        job_params2: fw::fragment::raw::JobParameters2 {
                            store_pipeline_bind: cmdbuf.store_pipeline_bind,
                            store_pipeline_addr: cmdbuf.store_pipeline | 4,
                            unk_8: 0x0,
                            unk_c: 0x0,
                            uuid1: 0x3b315cae,
                            uuid2: 0x3b6c7b92,
                            unk_18: U64(0x0),
                            tile_blocks_y: 4 * tile_info.tile_blocks_y as u16,
                            tile_blocks_x: 4 * tile_info.tile_blocks_x as u16,
                            unk_24: 0x0,
                            tile_counts: ((tile_info.tiles_y - 1) << 12) | (tile_info.tiles_x - 1),
                            unk_2c: aux_fb_unk,
                            depth_clear_val1: F32(cmdbuf.depth_clear_value),
                            stencil_clear_val1: cmdbuf.stencil_clear_value,
                            unk_35: 0x7, // clear flags? 2 : depth 4 : stencil?
                            unk_36: 0x0,
                            unk_38: 0x0,
                            unk_3c: 0x1,
                            unk_40: 0,
                        },
                        job_params3: fw::fragment::raw::JobParameters3::ver {
                            unk_44_padding: Default::default(),
                            depth_bias_array: fw::fragment::raw::ArrayAddr {
                                ptr: U64(cmdbuf.depth_bias_array),
                                unk_padding: U64(0),
                            },
                            scissor_array: fw::fragment::raw::ArrayAddr {
                                ptr: U64(cmdbuf.scissor_array),
                                unk_padding: U64(0),
                            },
                            unk_110: U64(0x0),
                            unk_118: U64(0x0),
                            unk_120: Default::default(),
                            unk_reload_pipeline: fw::fragment::raw::ClearPipelineBinding {
                                pipeline_bind: U64(cmdbuf.partial_reload_pipeline_bind as u64),
                                address: U64(cmdbuf.partial_reload_pipeline as u64 | 4),
                            },
                            unk_258: U64(0),
                            unk_260: U64(0),
                            unk_268: U64(0),
                            unk_270: U64(0),
                            reload_pipeline: fw::fragment::raw::ClearPipelineBinding {
                                pipeline_bind: U64(cmdbuf.partial_reload_pipeline_bind as u64),
                                address: U64(cmdbuf.partial_reload_pipeline as u64 | 4),
                            },
                            zls_ctrl: U64(cmdbuf.zls_ctrl as u64),
                            unk_290: U64(0x0),
                            depth_buffer_ptr1: U64(cmdbuf.depth_buffer_1),
                            unk_2a0: U64(0x0),
                            unk_2a8: U64(0x0),
                            depth_buffer_ptr2: U64(cmdbuf.depth_buffer_2),
                            depth_buffer_ptr3: U64(cmdbuf.depth_buffer_3),
                            unk_2c0: U64(0x0),
                            stencil_buffer_ptr1: U64(cmdbuf.stencil_buffer_1),
                            unk_2d0: U64(0x0),
                            unk_2d8: U64(0x0),
                            stencil_buffer_ptr2: U64(cmdbuf.stencil_buffer_2),
                            stencil_buffer_ptr3: U64(cmdbuf.stencil_buffer_3),
                            unk_2f0: Default::default(),
                            aux_fb_unk0: aux_fb_unk,
                            unk_30c: 0x0,
                            aux_fb_info: aux_fb_info,
                            unk_320_padding: Default::default(),
                            unk_partial_store_pipeline:
                                fw::fragment::raw::StorePipelineBinding::new(
                                    cmdbuf.partial_store_pipeline_bind,
                                    cmdbuf.partial_store_pipeline | 4
                                ),
                            partial_store_pipeline: fw::fragment::raw::StorePipelineBinding::new(
                                cmdbuf.partial_store_pipeline_bind,
                                cmdbuf.partial_store_pipeline | 4
                            ),
                            depth_clear_val2: F32(cmdbuf.depth_clear_value),
                            stencil_clear_val2: cmdbuf.stencil_clear_value,
                            unk_375: 3, // sometimes 1
                            unk_376: 0x0,
                            unk_378: unk_378,
                            unk_37c: 0x0,
                            unk_380: U64(0x0),
                            unk_388: U64(0x0),
                            #[ver(V >= V13_0B4)]
                            unk_390_0: 0x0,
                            depth_dimensions: U64(cmdbuf.depth_dimensions as u64),
                        },
                        unk_758_flag: 0,
                        unk_75c_flag: 0,
                        unk_buf: Default::default(),
                        busy_flag: 0,
                        tvb_overflow_count: 0,
                        unk_878: 0,
                        encoder_params: fw::job::EncoderParams {
                            unk_8: (cmdbuf.flags
                                & bindings::ASAHI_CMDBUF_SET_WHEN_RELOADING_Z_OR_S as u64
                                != 0) as u32,
                            unk_c: 0x0,  // fixed
                            unk_10: 0x0, // fixed
                            encoder_id: cmdbuf.encoder_id,
                            unk_18: 0x0, // fixed
                            unk_1c: 0xffffffff,
                            seq_buffer: inner.scene.seq_buf_pointer(),
                            unk_28: U64(0x0), // fixed
                            unk_30: unk_flag as u32,
                            unk_34: (cmdbuf.flags
                                & bindings::ASAHI_CMDBUF_NO_CLEAR_PIPELINE_TEXTURES as u64
                                != 0) as u32,
                            unk_38: 0, // 1 for boot stuff?
                        },
                        unk_pointee: 0,
                        meta: fw::job::JobMeta {
                            unk_4: 0,
                            stamp: batches_frag.event().stamp_pointer(),
                            fw_stamp: batches_frag.event().fw_stamp_pointer(),
                            stamp_value: next_frag,
                            stamp_slot: batches_frag.event().slot(),
                            unk_20: 0, // fixed
                            unk_24: unk0,
                            uuid: uuid_3d,
                            prev_stamp_value: batches_frag.event_value().counter(),
                            unk_30: unk1, // sometimes 1?
                            unk_buf_0: U64(0),
                            unk_buf_8: U64(0),
                            unk_buf_10: U64(0),
                            ts1: U64(0),
                            ts2: U64(0),
                            ts3: U64(0),
                        },
                        unk_914: 0,
                        unk_918: U64(0),
                        unk_920: 0,
                        unk_924: slot_client_seq,
                        unk_925: 0,
                        unk_926: 0,
                        unk_927: 0,
                        #[ver(V >= V13_0B4)]
                        unk_928_0: 0,
                        #[ver(V >= V13_0B4)]
                        unk_928_4: 0,
                        #[ver(V >= V13_0B4)]
                        ts_flag: 0,
                        #[ver(V >= V13_0B4)]
                        unk_5e6: 0,
                        #[ver(V >= V13_0B4)]
                        unk_5e8: Default::default(),
                    }
                ))
            },
        )?;

        if scene.rebind() {
            let bind_buffer = kalloc.private.new_inplace(
                fw::buffer::InitBuffer::ver {
                    scene: scene.clone(),
                },
                |_inner, ptr: *mut MaybeUninit<fw::buffer::raw::InitBuffer::ver>| {
                    Ok(place!(
                        ptr,
                        fw::buffer::raw::InitBuffer::ver {
                            tag: fw::workqueue::CommandType::InitBuffer,
                            vm_slot: vm_bind.slot(),
                            buffer_slot: scene.slot(),
                            unk_c: 0,
                            block_count: self.buffer.block_count(),
                            buffer: scene.buffer_pointer(),
                            stamp_value: next_vtx,
                        }
                    ))
                },
            )?;

            batches_vtx.add(Box::try_new(bind_buffer)?)?;
        }

        let vtx = GpuObject::new_prealloc(
            kalloc.private.prealloc()?,
            |ptr: GpuWeakPointer<fw::vertex::RunVertex::ver>| {
                let mut builder = microseq::Builder::new();

                let stats = inner_weak_ptr!(
                    gpu.initdata.runtime_pointers.stats.vtx.weak_pointer(),
                    stats
                );

                let start_vtx = builder.add(microseq::StartVertex::ver {
                    header: microseq::op::StartVertex::HEADER,
                    tiling_params: inner_weak_ptr!(ptr, tiling_params),
                    job_params1: inner_weak_ptr!(ptr, job_params1),
                    buffer: scene.buffer_pointer(),
                    scene: scene.weak_pointer(),
                    stats,
                    work_queue: self.wq_vtx.info_pointer(),
                    vm_slot: vm_bind.slot(),
                    unk_38: 1, // fixed
                    event_generation: self.id as u32,
                    buffer_slot: scene.slot(),
                    unk_44: 0,
                    prev_stamp_value: U64(batches_vtx.event_value().counter() as u64),
                    unk_50: 0,
                    unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                    unk_job_buf: inner_weak_ptr!(ptr, meta.unk_buf_0),
                    unk_64: 0x0,  // fixed
                    unk_68: unk1, // sometimes 1?
                    uuid: uuid_ta,
                    unk_70: 0x0,                // fixed
                    unk_74: Default::default(), // fixed
                    unk_15c: 0x0,               // fixed
                    unk_160: U64(0x0),          // fixed
                    unk_168: 0x0,               // fixed
                    unk_16c: 0x0,               // fixed
                    unk_170: U64(0x0),          // fixed
                    unk_178: 0x0,               // fixed?
                    #[ver(V >= V13_0B4)]
                    unk_17c: 0x0,
                    #[ver(V >= V13_0B4)]
                    notifier_buf: inner_weak_ptr!(&notifier.weak_pointer(), state.unk_buf),
                    #[ver(V >= V13_0B4)]
                    unk_188: 0x0,
                })?;

                builder.add(microseq::WaitForIdle {
                    header: microseq::op::WaitForIdle::new(microseq::Pipe::Vertex),
                })?;

                let off = builder.offset_to(start_vtx);
                builder.add(microseq::FinalizeVertex::ver {
                    header: microseq::op::FinalizeVertex::HEADER,
                    scene: scene.weak_pointer(),
                    buffer: scene.buffer_pointer(),
                    stats,
                    work_queue: self.wq_vtx.info_pointer(),
                    vm_slot: vm_bind.slot(),
                    unk_28: 0x0, // fixed
                    unk_pointer: inner_weak_ptr!(ptr, unk_pointee),
                    unk_34: 0x0, // fixed
                    uuid: uuid_ta,
                    fw_stamp: batches_vtx.event().fw_stamp_pointer(),
                    stamp_value: next_vtx,
                    unk_48: U64(0x0), // fixed
                    unk_50: 0x0,      // fixed
                    unk_54: 0x0,      // fixed
                    unk_58: U64(0x0), // fixed
                    unk_60: 0x0,      // fixed
                    unk_64: 0x0,      // fixed
                    unk_68: 0x0,      // fixed
                    restart_branch_offset: off,
                    unk_70: 0x0, // fixed
                    #[ver(V >= V13_0B4)]
                    unk_74: Default::default(), // Ventura
                })?;

                builder.add(microseq::RetireStamp {
                    header: microseq::op::RetireStamp::HEADER,
                })?;

                Ok(box_in_place!(fw::vertex::RunVertex::ver {
                    notifier: notifier.clone(),
                    scene: scene.clone(),
                    micro_seq: builder.build(&mut kalloc.private)?,
                    vm_bind: vm_bind.clone(),
                })?)
            },
            |inner, ptr| {
                Ok(place!(
                    ptr,
                    fw::vertex::raw::RunVertex::ver {
                        tag: fw::workqueue::CommandType::RunVertex,
                        #[ver(V >= V13_0B4)]
                        counter: 1,
                        vm_slot: vm_bind.slot(),
                        unk_8: 0,
                        notifier: inner.notifier.gpu_pointer(),
                        buffer_slot: inner.scene.slot(),
                        unk_1c: 0,
                        buffer: inner.scene.buffer_pointer(),
                        scene: inner.scene.gpu_pointer(),
                        unk_buffer_buf: inner.scene.kernel_buffer_pointer(),
                        unk_34: 0,
                        job_params1: fw::vertex::raw::JobParameters1 {
                            unk_0: U64(0x200),
                            unk_8: 0x1e3ce508, // fixed
                            unk_c: 0x1e3ce508, // fixed
                            tvb_tilemap: inner.scene.tvb_tilemap_pointer(),
                            unkptr_18: U64(0x0),
                            tvb_something: self.buffer.tvb_something_pointer(),
                            tvb_heapmeta: inner.scene.tvb_heapmeta_pointer().or(0x8000000000000000),
                            iogpu_unk_54: 0x6b0003, // fixed
                            iogpu_unk_55: 0x3a0012, // fixed
                            iogpu_unk_56: U64(0x1), // fixed
                            unk_40: U64(0x0),       // fixed
                            unk_48: U64(0xa000),    // fixed - maybe tvb_something_size?
                            unk_50: U64(0x88),      // fixed
                            tvb_heapmeta_2: inner.scene.tvb_heapmeta_pointer(),
                            unk_60: U64(0x0), // fixed
                            unk_68: U64(0x0), // fixed
                            preempt_buf1: inner_ptr!(inner.scene.preempt_buf_pointer(), part_1),
                            preempt_buf2: inner_ptr!(inner.scene.preempt_buf_pointer(), part_2),
                            unk_80: U64(0x1), // fixed
                            preempt_buf3: inner_ptr!(inner.scene.preempt_buf_pointer(), part_3)
                                .or(0x4000000000000), // check
                            encoder_addr: U64(cmdbuf.encoder_ptr),
                            unk_98: Default::default(), // fixed
                            unk_a8: U64(0xa041),        // fixed
                            unk_b0: Default::default(), // fixed
                            pipeline_base: U64(0x11_00000000),
                            unk_e8: U64(0x0),            // fixed
                            unk_f0: U64(0x1c),           // fixed
                            unk_f8: U64(0x8c60),         // fixed
                            unk_100: Default::default(), // fixed
                            unk_118: 0x1c,               // fixed
                        },
                        unk_154: Default::default(),
                        tiling_params: tile_info.params,
                        unk_3e8: Default::default(),
                        tvb_something: self.buffer.tvb_something_pointer(),
                        tvb_size: U64(0x800 * tile_info.tile_blocks as u64),
                        microsequence: inner.micro_seq.gpu_pointer(),
                        microsequence_size: inner.micro_seq.len() as u32,
                        fragment_stamp_slot: batches_frag.event().slot(),
                        fragment_stamp_value: next_frag,
                        unk_pointee: 0,
                        unk_pad: 0,
                        job_params2: fw::vertex::raw::JobParameters2 {
                            unk_480: Default::default(), // fixed
                            unk_498: U64(0x0),           // fixed
                            unk_4a0: 0x0,                // fixed
                            preempt_buf1: inner_ptr!(inner.scene.preempt_buf_pointer(), part_1),
                            unk_4ac: 0x0,      // fixed
                            unk_4b0: U64(0x0), // fixed
                            unk_4b8: 0x0,      // fixed
                            unk_4bc: U64(0x0), // fixed
                            unk_4c4_padding: Default::default(),
                            unk_50c: 0x0,      // fixed
                            unk_510: U64(0x0), // fixed
                            unk_518: U64(0x0), // fixed
                            unk_520: U64(0x0), // fixed
                        },
                        encoder_params: fw::job::EncoderParams {
                            unk_8: 0x0,  // fixed
                            unk_c: 0x0,  // fixed
                            unk_10: 0x0, // fixed
                            encoder_id: cmdbuf.encoder_id,
                            unk_18: 0x0, // fixed
                            unk_1c: 0xffffffff,
                            seq_buffer: inner.scene.seq_buf_pointer(),
                            unk_28: U64(0x0), // fixed
                            unk_30: 0,
                            unk_34: 0,
                            unk_38: 0, // 1 for boot stuff?
                        },
                        unk_568: 0,
                        unk_56c: 0,
                        meta: fw::job::JobMeta {
                            unk_4: 0,
                            stamp: batches_vtx.event().stamp_pointer(),
                            fw_stamp: batches_vtx.event().fw_stamp_pointer(),
                            stamp_value: next_vtx,
                            stamp_slot: batches_vtx.event().slot(),
                            unk_20: 0,    // fixed
                            unk_24: unk0, // 1 for boot stuff?
                            uuid: uuid_ta,
                            prev_stamp_value: batches_vtx.event_value().counter(),
                            unk_30: unk1, // sometimes 1?
                            unk_buf_0: U64(0),
                            unk_buf_8: U64(0),
                            unk_buf_10: U64(0),
                            ts1: U64(0),
                            ts2: U64(0),
                            ts3: U64(0),
                        },
                        unk_5c4: 0,
                        unk_5c8: 0,
                        unk_5cc: 0,
                        unk_5d0: 0,
                        unk_5d4: slot_client_seq,
                        pad_5d5: Default::default(),
                        #[ver(V >= V13_0B4)]
                        unk_5e0: 0,
                        #[ver(V >= V13_0B4)]
                        unk_5e4: 0,
                        #[ver(V >= V13_0B4)]
                        ts_flag: 0,
                        #[ver(V >= V13_0B4)]
                        unk_5e6: 0,
                        #[ver(V >= V13_0B4)]
                        unk_5e8: Default::default(),
                        pad_5d8: Default::default(),
                    }
                ))
            },
        )?;

        notifier.threshold.with(|raw, _inner| {
            raw.increment();
        });
        batches_frag.add(Box::try_new(frag)?)?;
        let batch_frag = batches_frag.commit()?;

        notifier.threshold.with(|raw, _inner| {
            raw.increment();
        });
        batches_vtx.add(Box::try_new(vtx)?)?;
        let batch_vtx = batches_vtx.commit()?;

        mod_dev_dbg!(self.dev, "[Submission {}] Submit frag!\n", id);
        gpu.submit_batch(batches_frag)?;
        mod_dev_dbg!(self.dev, "[Submission {}] Submit vert!\n", id);
        gpu.submit_batch(batches_vtx)?;

        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Waiting for vertex batch...\n",
            id
        );
        batch_vtx.wait();
        mod_dev_dbg!(self.dev, "[Submission {}] Vertex batch completed!\n", id);
        mod_dev_dbg!(
            self.dev,
            "[Submission {}] Waiting for fragment batch...\n",
            id
        );
        batch_frag.wait();
        mod_dev_dbg!(self.dev, "[Submission {}] Fragment batch completed!\n", id);

        if debug_enabled(debug::DebugFlags::WaitForPowerOff) {
            mod_dev_dbg!(self.dev, "[Submission {}] Waiting for GPU power-off\n", id);
            if gpu.wait_for_poweroff(100).is_err() {
                dev_warn!(self.dev, "[Submission {}] GPU failed to power off\n", id);
            }
            mod_dev_dbg!(self.dev, "[Submission {}] GPU powered off\n", id);
        }

        Ok(())
    }
}

#[versions(AGX)]
impl Drop for Renderer::ver {
    fn drop(&mut self) {
        let dev = self.dev.data();
        if dev.gpu.invalidate_context(&self.gpu_context).is_err() {
            dev_err!(
                self.dev,
                "Renderer::drop: Failed to invalidate GPU context!\n"
            );
        }
    }
}
