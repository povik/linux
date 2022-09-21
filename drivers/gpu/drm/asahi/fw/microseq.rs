// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! GPU events & stamps

use super::types::*;
use super::{buffer, fragment, initdata, vertex, workqueue};

pub(crate) trait Operation {}

#[derive(Debug, Copy, Clone)]
#[repr(u32)]
enum OpCode {
    WaitForIdle = 0x01,
    RetireStamp = 0x18,
    Timestamp = 0x19,
    StartVertex = 0x22,
    FinalizeVertex = 0x23,
    StartFragment = 0x24,
    FinalizeFragment = 0x25,
    StartCompute = 0x29,
    FinalizeCompute = 0x2a,
}

#[derive(Debug, Copy, Clone)]
#[repr(u32)]
pub(crate) enum Pipe {
    Vertex = 1 << 0,
    Fragment = 1 << 8,
    Compute = 1 << 15,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct OpHeader(u32);

impl OpHeader {
    const fn new(opcode: OpCode) -> OpHeader {
        OpHeader(opcode as u32)
    }
    const fn with_args(opcode: OpCode, args: u32) -> OpHeader {
        OpHeader(opcode as u32 | args)
    }
}

macro_rules! simple_op {
    ($name:ident) => {
        #[derive(Debug, Copy, Clone)]
        pub(crate) struct $name(OpHeader);

        impl $name {
            const HEADER: $name = $name(OpHeader::new(OpCode::$name));
        }
    };
}

pub(crate) mod op {
    use super::*;

    simple_op!(RetireStamp);
    simple_op!(StartVertex);
    simple_op!(FinalizeVertex);
    simple_op!(StartFragment);
    simple_op!(FinalizeFragment);
    simple_op!(StartCompute);
    simple_op!(FinalizeCompute);

    #[derive(Debug, Copy, Clone)]
    pub(crate) struct WaitForIdle(OpHeader);
    impl WaitForIdle {
        pub(crate) const fn new(pipe: Pipe) -> WaitForIdle {
            WaitForIdle(OpHeader::with_args(OpCode::WaitForIdle, (pipe as u32) << 8))
        }
    }

    #[derive(Debug, Copy, Clone)]
    pub(crate) struct Timestamp(OpHeader);
    impl Timestamp {
        pub(crate) const fn new(flag: bool) -> Timestamp {
            Timestamp(OpHeader::with_args(OpCode::Timestamp, (flag as u32) << 31))
        }
    }
}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
struct StartVertex<'a> {
    pub(crate) header: op::StartVertex,
    pub(crate) tiling_params: GpuWeakPointer<vertex::raw::TilingParameters>,
    pub(crate) job_params: GpuWeakPointer<vertex::raw::JobParameters1<'a>>,
    pub(crate) buffer: GpuWeakPointer<buffer::Info::ver>,
    pub(crate) scene: GpuWeakPointer<buffer::Scene::ver>,
    pub(crate) stats_ptr: GpuWeakPointer<initdata::raw::GpuStatsVtx::ver>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_38: u32,
    pub(crate) event_generation: u32,
    pub(crate) buffer_slot: u64,
    pub(crate) unk_48: u64,
    pub(crate) unk_50: u32,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) unk_job_buf: GpuWeakPointer<Array<0x18, u8>>,
    pub(crate) unk_64: u32,
    pub(crate) unk_68: u32,
    pub(crate) uuid: u32,
    pub(crate) unk_70: u32,
    pub(crate) unk_74: Array<0x1d, u64>,
    pub(crate) unk_15c: u32,
    pub(crate) unk_160: u64,
    pub(crate) unk_168: u32,
    pub(crate) unk_16c: u32,
    pub(crate) unk_170: u64,
    pub(crate) unk_178: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_17c: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) notifier_buf: GpuWeakPointer<Array<0x8, u8>>,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_188: u32,
}

#[versions(AGX)]
impl<'a> Operation for StartVertex::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
struct FinalizeVertex {
    pub(crate) opcode: op::FinalizeVertex,
    pub(crate) scene: GpuWeakPointer<buffer::Scene::ver>,
    pub(crate) buffer: GpuWeakPointer<buffer::Info::ver>,
    pub(crate) stats_ptr: GpuWeakPointer<initdata::raw::GpuStatsVtx::ver>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo>,
    pub(crate) vm_slot: u32,
    pub(crate) unk_28: u32,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) unk_34: u32,
    pub(crate) uuid: u32,
    pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
    pub(crate) stamp_value: u32,
    pub(crate) unk_48: u64,
    pub(crate) unk_50: u32,
    pub(crate) unk_54: u32,
    pub(crate) unk_58: U64,
    pub(crate) unk_60: u32,
    pub(crate) unk_64: u32,
    pub(crate) unk_68: u32,
    pub(crate) restart_branch_offset: i32,
    pub(crate) unk_70: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_74: Array<0x10, u8>,
}

#[versions(AGX)]
impl Operation for FinalizeVertex::ver {}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub(crate) struct Attachment {
    pub(crate) address: U64,
    pub(crate) size: u32,
    pub(crate) unk_c: u16,
    pub(crate) unk_e: u16,
}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
struct StartFragment<'a> {
    pub(crate) opcode: op::StartFragment,
    pub(crate) job_params2: U64,
    pub(crate) job_params1: U64,
    pub(crate) scene: GpuPointer<'a, buffer::Scene::ver>,
    pub(crate) stats_ptr: U64,
    pub(crate) busy_flag_ptr: GpuWeakPointer<u32>,
    pub(crate) tvb_overflow_count: GpuWeakPointer<u32>,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) cmdqueue_ptr: U64,
    pub(crate) workitem_ptr: U64,
    pub(crate) context_id: u32,
    pub(crate) unk_50: u32,
    pub(crate) event_generation: u32,
    pub(crate) buffer_mgr_slot: u32,
    pub(crate) unk_5c: u32,
    pub(crate) prev_stamp_value: U64,
    pub(crate) unk_68: u32,
    pub(crate) unk_buf_ptr: U64,
    pub(crate) unk_buf2_ptr: U64,
    pub(crate) unk_7c: u32,
    pub(crate) unk_80: u32,
    pub(crate) unk_84: u32,
    pub(crate) uuid: u32,
    pub(crate) attachments: Array<0x10, Attachment>,
    pub(crate) num_attachments: u32,
    pub(crate) unk_190: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_194: U64,

    #[ver(V >= V13_0B4)]
    pub(crate) unkptr_19c: U64,
}

#[versions(AGX)]
impl<'a> Operation for StartFragment::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
struct FinalizeFragment {
    pub(crate) opcode: op::FinalizeFragment,
    pub(crate) uuid: u32,
    pub(crate) unk_8: u32,
    pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
    pub(crate) stamp_value: u32,
    pub(crate) unk_18: u32,
    pub(crate) scene: GpuWeakPointer<buffer::Scene::ver>,
    pub(crate) buffer: GpuWeakPointer<buffer::Info::ver>,
    pub(crate) unk_2c: U64,
    pub(crate) stats_ptr: GpuWeakPointer<initdata::raw::GpuStatsFrag::ver>,
    pub(crate) unk_pointer: GpuWeakPointer<u32>,
    pub(crate) busy_flag: GpuWeakPointer<u32>,
    pub(crate) work_queue: GpuWeakPointer<workqueue::QueueInfo>,
    pub(crate) work_item: GpuWeakPointer<fragment::RunFragment::ver>,
    pub(crate) unk_5c: U64,
    pub(crate) unk_758_ptr: GpuWeakPointer<u32>,
    pub(crate) unk_6c: U64,
    pub(crate) unk_74: U64,
    pub(crate) unk_7c: U64,
    pub(crate) unk_84: U64,
    pub(crate) unk_8c: U64,
    pub(crate) restart_branch_offset: i32,
    pub(crate) unk_98: u32,

    #[ver(V >= V13_0B4)]
    pub(crate) unk_9c: Array<0x10, u8>,
}

#[versions(AGX)]
impl Operation for FinalizeFragment::ver {}
