// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! GPU events & stamps

use super::types::*;
use super::{buffer, initdata, vertex, workqueue};

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
    header: op::StartVertex,
    tiling_params: GpuWeakPointer<vertex::raw::TilingParameters>,
    job_params: GpuWeakPointer<vertex::raw::JobParameters<'a>>,
    buffer: GpuWeakPointer<buffer::Info::ver>,
    scene: GpuWeakPointer<buffer::Scene::ver>,
    stats_ptr: GpuWeakPointer<initdata::raw::GpuStatsVtx::ver>,
    work_queue: GpuWeakPointer<workqueue::QueueInfo>,
    vm_slot: u32,
    unk_38: u32,
    event_generation: u32,
    buffer_slot: u64,
    unk_48: u64,
    unk_50: u32,
    job_meta: GpuWeakPointer<vertex::raw::JobMeta<'a>>,
    unk_job_buf: GpuWeakPointer<Array<0x18, u8>>,
    unk_64: u32,
    unk_68: u32,
    uuid: u32,
    unk_70: u32,
    unk_74: Array<0x1d, u64>,
    unk_15c: u32,
    unk_160: u64,
    unk_168: u32,
    unk_16c: u32,
    unk_170: u64,
    unk_178: u32,

    #[ver(V >= V13_0B4)]
    unk_17c: u32,

    #[ver(V >= V13_0B4)]
    notifier_buf: GpuWeakPointer<Array<0x8, u8>>,

    #[ver(V >= V13_0B4)]
    unk_188: u32,
}

#[versions(AGX)]
impl<'a> Operation for StartVertex::ver<'a> {}

#[versions(AGX)]
#[derive(Debug)]
#[repr(C)]
struct FinalizeVertex<'a> {
    opcode: u32,
    scene: GpuWeakPointer<buffer::Scene::ver>,
    buffer: GpuWeakPointer<buffer::Info::ver>,
    stats_ptr: GpuWeakPointer<initdata::raw::GpuStatsVtx::ver>,
    work_queue: GpuWeakPointer<workqueue::QueueInfo>,
    vm_slot: u32,
    unk_28: u32,
    job_meta: GpuWeakPointer<vertex::raw::JobMeta<'a>>,
    unk_34: u32,
    uuid: u32,
    fw_stamp: GpuWeakPointer<FwStamp>,
    stamp_value: u32,
    unk_48: u64,
    unk_50: u32,
    unk_54: u32,
    unk_58: U64,
    unk_60: u32,
    unk_64: u32,
    unk_68: u32,
    restart_branch_offset: i32,
    unk_70: u32,

    #[ver(V >= V13_0B4)]
    unk_74: Array<0x10, u8>,
}

#[versions(AGX)]
impl<'a> Operation for FinalizeVertex::ver<'a> {}
