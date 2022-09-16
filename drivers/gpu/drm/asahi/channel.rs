// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]

//! Asahi ring buffer channels

use crate::fw::channels::*;
use crate::fw::initdata::{raw, ChannelRing};
use crate::fw::types::*;
use crate::gpu;
use crate::object::GpuStruct;
use kernel::{dbg, prelude::*};

pub(crate) struct RxChannel<T: RxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    ring: ChannelRing<T, U>,
    rptr: u32,
    count: u32,
}

impl<T: RxChannelState, U: Copy + Default> RxChannel<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators, count: usize) -> Result<RxChannel<T, U>> {
        Ok(RxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.shared.array_empty(count)?,
            },
            rptr: 0,
            count: count as u32,
        })
    }

    pub(crate) fn get(&mut self) -> Option<U> {
        self.ring.state.with(|raw, _inner| {
            let wptr = T::wptr(raw);
            if wptr == self.rptr {
                None
            } else {
                let msg = self.ring.ring.as_slice()[self.rptr as usize];
                self.rptr = (self.rptr + 1) % self.count;
                T::set_rptr(raw, self.rptr);
                Some(msg)
            }
        })
    }
}

pub(crate) struct StatsChannel {
    ch: RxChannel<ChannelState, StatsMsg>,
}

impl StatsChannel {
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<StatsChannel> {
        Ok(StatsChannel {
            ch: RxChannel::<ChannelState, StatsMsg>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, StatsMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get() {
            dbg!(msg);
        }
    }
}
