// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]

//! Asahi ring buffer channels

use crate::fw::channels::*;
use crate::fw::initdata::{raw, ChannelRing};
use crate::fw::types::*;
use crate::{event, gpu};
use core::time::Duration;
use kernel::{dbg, delay::coarse_sleep, prelude::*, sync::Arc};

pub(crate) use crate::fw::channels::PipeType;

pub(crate) struct RxChannel<T: RxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    ring: ChannelRing<T, U>,
    // FIXME: needs feature(generic_const_exprs)
    //rptr: [u32; T::SUB_CHANNELS],
    rptr: [u32; 6],
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
            rptr: Default::default(),
            count: count as u32,
        })
    }

    pub(crate) fn get(&mut self, index: usize) -> Option<U> {
        self.ring.state.with(|raw, _inner| {
            let wptr = T::wptr(raw, index);
            let rptr = &mut self.rptr[index];
            if wptr == *rptr {
                None
            } else {
                let msg = self.ring.ring[*rptr as usize];
                *rptr = (*rptr + 1) % self.count;
                T::set_rptr(raw, index, *rptr);
                Some(msg)
            }
        })
    }
}

pub(crate) struct TxChannel<T: TxChannelState, U: Copy + Default>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    ring: ChannelRing<T, U>,
    wptr: u32,
    count: u32,
}

impl<T: TxChannelState, U: Copy + Default> TxChannel<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug + Default,
{
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators, count: usize) -> Result<TxChannel<T, U>> {
        Ok(TxChannel {
            ring: ChannelRing {
                state: alloc.shared.new_default()?,
                ring: alloc.private.array_empty(count)?,
            },
            wptr: 0,
            count: count as u32,
        })
    }

    pub(crate) fn put(&mut self, msg: &U) -> u32 {
        self.ring.state.with(|raw, _inner| {
            let next_wptr = (self.wptr + 1) % self.count;
            let mut rptr = T::rptr(raw);
            if next_wptr == rptr {
                pr_err!(
                    "TX ring buffer is full! Waiting... ({}, {})",
                    next_wptr,
                    rptr
                );
                // TODO: block properly on incoming messages?
                while next_wptr == rptr {
                    coarse_sleep(Duration::from_millis(8));
                    rptr = T::rptr(raw);
                }
            }
            self.ring.ring[self.wptr as usize] = *msg;
            T::set_wptr(raw, next_wptr);
            self.wptr = next_wptr;
        });
        self.wptr
    }

    pub(crate) fn wait_for(&mut self, wptr: u32, timeout_ms: usize) -> Result {
        self.ring.state.with(|raw, _inner| {
            for _i in 0..timeout_ms {
                if T::rptr(raw) == wptr {
                    return Ok(());
                }
                coarse_sleep(Duration::from_millis(1));
            }
            Err(ETIMEDOUT)
        })
    }
}

pub(crate) struct DeviceControlChannel {
    ch: TxChannel<ChannelState, DeviceControlMsg>,
}

impl DeviceControlChannel {
    const COMMAND_TIMEOUT_MS: usize = 100;

    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<DeviceControlChannel> {
        Ok(DeviceControlChannel {
            ch: TxChannel::<ChannelState, DeviceControlMsg>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, DeviceControlMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn send(&mut self, msg: &DeviceControlMsg) -> u32 {
        self.ch.put(msg)
    }

    pub(crate) fn wait_for(&mut self, wptr: u32) -> Result {
        self.ch.wait_for(wptr, Self::COMMAND_TIMEOUT_MS)
    }
}

pub(crate) struct PipeChannel {
    ch: TxChannel<ChannelState, PipeMsg>,
}

impl PipeChannel {
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<PipeChannel> {
        Ok(PipeChannel {
            ch: TxChannel::<ChannelState, PipeMsg>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, PipeMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn send(&mut self, msg: &PipeMsg) {
        self.ch.put(msg);
    }
}

pub(crate) struct EventChannel {
    ch: RxChannel<ChannelState, RawEventMsg>,
    mgr: Arc<event::EventManager>,
}

impl EventChannel {
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
        mgr: Arc<event::EventManager>,
    ) -> Result<EventChannel> {
        Ok(EventChannel {
            ch: RxChannel::<ChannelState, RawEventMsg>::new(alloc, 0x100)?,
            mgr,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawEventMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            let tag = unsafe { msg.raw.0 };
            match tag {
                0..=EVENT_MAX => {
                    let msg = unsafe { msg.msg };
                    match msg {
                        EventMsg::Fault => {
                            pr_crit!("GPU faulted!");
                        }
                        EventMsg::Timeout { .. } => {
                            pr_crit!("GPU timeout! {:?}", msg);
                        }
                        EventMsg::Flag { firing, .. } => {
                            pr_crit!("GPU flag event: {:?}", msg);
                            for (i, flags) in firing.iter().enumerate() {
                                for j in 0..32 {
                                    if flags & (1u32 << j) != 0 {
                                        self.mgr.signal((i * 32 + j) as u32);
                                    }
                                }
                            }
                        }
                        msg => {
                            pr_crit!("Unknown event message: {:?}", msg);
                        }
                    }
                }
                _ => {
                    pr_warn!("Unknown event message: {:?}", unsafe { msg.raw });
                }
            }
        }
    }
}

pub(crate) struct FwLogChannel {
    ch: RxChannel<FwLogChannelState, RawFwLogMsg>,
}

impl FwLogChannel {
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<FwLogChannel> {
        Ok(FwLogChannel {
            ch: RxChannel::<FwLogChannelState, RawFwLogMsg>::new(alloc, 0x600)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<FwLogChannelState, RawFwLogMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        for i in 0..=FwLogChannelState::SUB_CHANNELS - 1 {
            while let Some(msg) = self.ch.get(i) {
                pr_info!("FwLog{}: {:?}", i, msg);
            }
        }
    }
}

pub(crate) struct KTraceChannel {
    ch: RxChannel<ChannelState, RawKTraceMsg>,
}

impl KTraceChannel {
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<KTraceChannel> {
        Ok(KTraceChannel {
            ch: RxChannel::<ChannelState, RawKTraceMsg>::new(alloc, 0x200)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawKTraceMsg> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            pr_info!("KTrace: {:?}", msg);
        }
    }
}

#[versions(AGX)]
pub(crate) struct StatsChannel {
    ch: RxChannel<ChannelState, RawStatsMsg::ver>,
}

#[versions(AGX)]
impl StatsChannel::ver {
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<StatsChannel::ver> {
        Ok(StatsChannel::ver {
            ch: RxChannel::<ChannelState, RawStatsMsg::ver>::new(alloc, 0x100)?,
        })
    }

    pub(crate) fn to_raw(&self) -> raw::ChannelRing<ChannelState, RawStatsMsg::ver> {
        self.ch.ring.to_raw()
    }

    pub(crate) fn poll(&mut self) {
        while let Some(msg) = self.ch.get(0) {
            let tag = unsafe { msg.raw.0 };
            match tag {
                0..=STATS_MAX::ver => {
                    //let msg = unsafe { msg.msg };
                    //pr_info!("Stats: {:?}", msg);
                }
                _ => {
                    pr_warn!("Unknown stats message: {:?}", unsafe { msg.raw });
                }
            }
        }
    }
}
