// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! GPU communication channels (ring buffers)

use super::types::*;
use super::{event, workqueue};
use crate::{buffer, fw};
use kernel::sync::Arc;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct TilingParameters {
        size1: u32,
        unk_4: u32,
        unk_8: u32,
        x_max: u16,
        y_max: u16,
        tile_count: u32,
        x_blocks: u32,
        y_blocks: u32,
        size2: u32,
        size3: u32,
        unk_24: u32,
        unk_28: u32,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobParameters<'a> {
        unk_0: U64,
        unk_8: u32,
        unk_c: u32,
        tvb_tilemap: GpuPointer<'a, &'a [u8]>,
        unkptr_18: U64,
        unkptr_20: U64,
        tvb_heapmeta_addr: GpuPointer<'a, &'a [u8]>,
        iogpu_unk_54: u32,
        iogpu_unk_55: u32,
        iogpu_unk_56: U64,
        unk_40: U64,
        unk_48: U64,
        unk_50: U64,
        tvb_heapmeta_addr2: GpuPointer<'a, &'a [u8]>,
        unk_60: U64,
        unk_68: U64,
        preempt_buf1: GpuPointer<'a, &'a [u8]>,
        preempt_buf2: GpuPointer<'a, &'a [u8]>,
        unk_80: U64,
        preempt_buf3: GpuPointer<'a, &'a [u8]>,
        encoder_addr: U64,
        unk_98: Array<2, U64>,
        unk_a8: U64,
        unk_b0: Array<6, U64>,
        pipeline_base: U64,
        unk_e8: U64,
        unk_f0: U64,
        unk_f8: U64,
        unk_100: Array<3, U64>,
        unk_118: u32,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobMeta<'a> {
        unk_480: Array<6, u32>,
        unk_498: U64,
        unk_4a0: u32,
        preempt_buf1: GpuPointer<'a, &'a [u8]>,
        unk_4ac: u32,
        unk_4b0: U64,
        unk_4b8: u32,
        unk_4bc: U64,
        unk_4c4_padding: Array<0x48, u8>,
        unk_50c: u32,
        unk_510: U64,
        unk_518: U64,
        unk_520: U64,
        unk_528: u32,
        unk_52c: u32,
        unk_530: u32,
        encoder_id: u32,
        unk_538: u32,
        unk_53c: u32,
        seq_buffer: GpuWeakPointer<[u8]>,
        unk_548: U64,
        unk_550: Array<6, u32>,
        stamp: GpuWeakPointer<Stamp>,
        fw_stamp: GpuWeakPointer<FwStamp>,
        stamp_value: u32,
        stamp_slot: u32,
        unk_580: u32,
        unk_584: u32,
        uuid: u32,
        prev_stamp_value: u32,
        unk_590: u32,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct RunVertex<'a> {
        tag: workqueue::CommandType,

        #[ver(V >= V13_0B4)]
        counter: U64,

        context_id: u32,
        unk_8: u32,
        notifier: GpuPointer<'a, event::Notifier>,
        buffer_slot: U64,
        buffer: GpuPointer<'a, fw::buffer::Info::ver>,
        scene: GpuPointer<'a, fw::buffer::Scene::ver>,
        unk_scene_buf: GpuPointer<'a, [u8]>,
        unk_34: u32,
        job_params: JobParameters<'a>,
        unk_154: Array<0x268, u8>,
        tiling_params: TilingParameters,
        unk_3e8: Array<0x74, u8>,
        unkptr_45c: U64,
        tvb_size: U64,
        microsequence_ptr: GpuPointer<'a, &'a [u8]>,
        microsequence_size: u32,
        fragment_stamp_slot: u32,
        stamp_value: u32,
        meta: JobMeta<'a>,
        unk_job_buf: Array<0x18, u8>,
        ts1: U64,
        ts2: U64,
        ts3: U64,
        unk_5c4: u32,
        unk_5c8: u32,
        unk_5cc: u32,
        unk_5d0: u32,
        unk_5d4: u8,
        pad_5d5: Array<3, u8>,

        #[ver(V >= V13_0B4)]
        unk_5e0: u32,

        #[ver(V >= V13_0B4)]
        unk_5e4: u8,

        #[ver(V >= V13_0B4)]
        ts_flag: u8,

        #[ver(V >= V13_0B4)]
        unk_5e6: u16,

        #[ver(V >= V13_0B4)]
        unk_5e8: [u8; 0x18],

        pad_5d8: Pad<0x8>,
        // Alignment - handled by allocator
        //#[ver(V >= V13_0B4)]
        //pad_5e0: Pad<0x18>,
    }
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct RunVertex {
    pub(crate) notifier: Arc<GpuObject<event::Notifier>>,
    pub(crate) scene: Arc<buffer::Scene::ver>,
}

#[versions(AGX)]
impl GpuStruct for RunVertex::ver {
    type Raw<'a> = raw::RunVertex::ver<'a>;
}
