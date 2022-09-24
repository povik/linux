// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi ring buffer channels

use crate::fw::event::*;
use crate::fw::initdata::raw;
use crate::fw::types::*;
use crate::{gpu, slotalloc, workqueue};
use core::cmp;
use core::sync::atomic::Ordering;
use kernel::sync::Arc;
use kernel::{dbg, prelude::*};

const NUM_EVENTS: u32 = 128;

pub(crate) struct EventInner {
    stamp: *const AtomicU32,
    gpu_stamp: GpuWeakPointer<Stamp>,
    gpu_fw_stamp: GpuWeakPointer<FwStamp>,
    owner: Option<Arc<workqueue::WorkQueue>>,
}

pub(crate) type Token = slotalloc::SlotToken;
pub(crate) type Event = slotalloc::Guard<EventInner>;

#[derive(Eq, PartialEq, Copy, Clone)]
pub(crate) struct EventValue(u32);

impl EventValue {
    pub(crate) fn stamp(&self) -> u32 {
        self.0
    }

    pub(crate) fn counter(&self) -> u32 {
        self.0 >> 8
    }

    pub(crate) fn next(&self) -> EventValue {
        EventValue(self.0.wrapping_add(0x100))
    }

    pub(crate) fn increment(&mut self) {
        self.0 = self.0.wrapping_add(0x100);
    }

    pub(crate) fn delta(&self, other: &EventValue) -> i32 {
        self.0.wrapping_sub(other.0) as i32
    }
}

impl PartialOrd for EventValue {
    fn partial_cmp(&self, other: &Self) -> Option<cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for EventValue {
    fn cmp(&self, other: &Self) -> cmp::Ordering {
        self.delta(other).cmp(&0)
    }
}

impl EventInner {
    pub(crate) fn stamp_pointer(&self) -> GpuWeakPointer<Stamp> {
        self.gpu_stamp
    }

    pub(crate) fn fw_stamp_pointer(&self) -> GpuWeakPointer<FwStamp> {
        self.gpu_fw_stamp
    }

    pub(crate) fn current(&self) -> EventValue {
        // SAFETY: The pointer is always valid as constructed in
        // EventManager below, and outside users cannot construct
        // new EventInners, nor move or copy them, and Guards as
        // returned by the SlotAllocator hold a reference to the
        // SlotAllocator containing the EventManagerInner, which
        // keeps the GpuObject the stamp is contained within alive.
        EventValue(unsafe { &*self.stamp }.load(Ordering::Acquire))
    }
}

impl slotalloc::SlotItem for EventInner {
    type Owner = EventManagerInner;

    fn release(&mut self, _owner: &mut Self::Owner, _slot: u32) {
        self.owner = None;
    }
}

pub(crate) struct EventManagerInner {
    stamps: GpuArray<Stamp>,
    fw_stamps: GpuArray<FwStamp>,
}

pub(crate) struct EventManager(slotalloc::SlotAllocator<EventInner>);

impl EventManager {
    pub(crate) fn new(alloc: &mut gpu::KernelAllocators) -> Result<EventManager> {
        let inner = EventManagerInner {
            stamps: alloc.shared.array_empty(NUM_EVENTS as usize)?,
            fw_stamps: alloc.private.array_empty(NUM_EVENTS as usize)?,
        };

        Ok(EventManager(slotalloc::SlotAllocator::new(
            NUM_EVENTS,
            inner,
            |inner: &mut EventManagerInner, slot| EventInner {
                stamp: &inner.stamps[slot as usize].0,
                gpu_stamp: inner.stamps.weak_item_pointer(slot as usize),
                gpu_fw_stamp: inner.fw_stamps.weak_item_pointer(slot as usize),
                owner: None,
            },
        )?))
    }

    pub(crate) fn get(&self, token: Option<Token>) -> Result<Event> {
        Ok(self.0.get(token)?)
    }
}
