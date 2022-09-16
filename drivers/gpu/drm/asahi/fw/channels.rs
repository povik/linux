// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! GPU communication channels (ring buffers)

use super::types::*;
use core::sync::atomic::Ordering;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct ChannelState<'a> {
        pub(crate) read_ptr: AtomicU32,
        __pad0: Pad<0x1c>,
        pub(crate) write_ptr: AtomicU32,
        __pad1: Pad<0xc>,
        _p: PhantomData<&'a ()>,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct FwCtlChannelState<'a> {
        pub(crate) read_ptr: AtomicU32,
        __pad0: Pad<0xc>,
        pub(crate) write_ptr: AtomicU32,
        __pad1: Pad<0xc>,
        _p: PhantomData<&'a ()>,
    }
}

pub(crate) trait RxChannelState: GpuStruct + Debug + Default {
    fn wptr<'a>(raw: &Self::Raw<'a>) -> u32;
    fn set_rptr<'a>(raw: &Self::Raw<'a>, rptr: u32);
}

#[derive(Debug, Default)]
pub(crate) struct ChannelState {}

impl GpuStruct for ChannelState {
    type Raw<'a> = raw::ChannelState<'a>;
}

impl RxChannelState for ChannelState {
    fn wptr<'a>(raw: &Self::Raw<'a>) -> u32 {
        raw.write_ptr.load(Ordering::Acquire)
    }

    fn set_rptr<'a>(raw: &Self::Raw<'a>, rptr: u32) {
        raw.read_ptr.store(rptr, Ordering::Release);
    }
}

#[derive(Debug, Default)]
pub(crate) struct FwLogChannelState {}

impl FwLogChannelState {
    const SUB_CHANNELS: usize = 6;
}

impl GpuStruct for FwLogChannelState {
    type Raw<'a> = Array<{ Self::SUB_CHANNELS }, raw::ChannelState<'a>>;
}

#[derive(Debug, Default)]
pub(crate) struct FwCtlChannelState {}

impl GpuStruct for FwCtlChannelState {
    type Raw<'a> = raw::FwCtlChannelState<'a>;
}

pub(crate) type RunCmdQueueMsg = Array<0x30, u8>;
pub(crate) type DeviceControlMsg = Array<0x30, u8>;
pub(crate) type EventMsg = Array<0x38, u8>;
pub(crate) type FwLogMsg = Array<0xd8, u8>;
pub(crate) type KTraceMsg = Array<0x38, u8>;
pub(crate) type StatsMsg = Array<0x60, u8>;
pub(crate) type FwCtlMsg = Array<0x14, u8>;
