// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! Common job structures

use super::types::*;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub(crate) struct JobMeta {
    unk_4: u32,
    stamp: GpuWeakPointer<Stamp>,
    fw_stamp: GpuWeakPointer<FwStamp>,
    stamp_value: u32,
    stamp_slot: u32,
    unk_20: u32,
    unk_24: u32,
    uuid: u32,
    prev_stamp_value: u32,
    unk_30: u32,
    unk_buf_0: U64,
    unk_buf_8: U64,
    unk_buf_10: U64,
    ts1: U64,
    ts2: U64,
    ts3: U64,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub(crate) struct EncoderParams {
    unk_8: u32,
    unk_c: u32,
    unk_10: u32,
    encoder_id: u32,
    unk_18: u32,
    unk_1c: u32,
    unknown_buffer: U64,
    unk_28: U64,
    unk_30: u32,
    unk_34: U64,
}
