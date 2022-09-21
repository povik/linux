// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Generic slot allocator

use core::ops::{Deref, DerefMut};
use kernel::{
    error::Result,
    prelude::*,
    sync::{Arc, CondVar, Mutex, UniqueArc},
};

pub(crate) trait SlotItem {
    type Owner;

    fn release(&mut self) {}
}

pub(crate) struct Guard<T: SlotItem> {
    item: Option<T>,
    slot: u32,
    alloc: Arc<SlotAllocatorOuter<T>>,
}

impl<T: SlotItem> Deref for Guard<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.item.as_ref().expect("SlotItem Guard lost our item!")
    }
}

impl<T: SlotItem> DerefMut for Guard<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.item.as_mut().expect("SlotItem Guard lost our item!")
    }
}

struct SlotAllocatorInner<T: SlotItem> {
    owner: T::Owner,
    slots: Vec<(Option<T>, u64)>,
    now: u64,
}

pub(crate) struct SlotAllocatorOuter<T: SlotItem> {
    inner: Mutex<SlotAllocatorInner<T>>,
    cond: CondVar,
}

pub(crate) struct SlotAllocator<T: SlotItem>(Arc<SlotAllocatorOuter<T>>);

impl<T: SlotItem> SlotAllocator<T> {
    pub(crate) fn new(
        num_slots: u32,
        mut owner: T::Owner,
        mut constructor: impl FnMut(&mut T::Owner, u32) -> T,
    ) -> Result<SlotAllocator<T>> {
        let mut slots = Vec::try_with_capacity(num_slots as usize)?;

        for i in 0..num_slots {
            slots
                .try_push((Some(constructor(&mut owner, i)), 0))
                .expect("try_push() failed after reservation");
        }

        let inner = SlotAllocatorInner {
            owner,
            slots,
            now: 0,
        };

        let mut alloc = Pin::from(UniqueArc::try_new(SlotAllocatorOuter {
            // SAFETY: `condvar_init!` is called below.
            cond: unsafe { CondVar::new() },
            // SAFETY: `mutex_init!` is called below.
            inner: unsafe { Mutex::new(inner) },
        })?);

        // SAFETY: `cond` is pinned when `alloc` is.
        let pinned = unsafe { alloc.as_mut().map_unchecked_mut(|s| &mut s.cond) };
        kernel::condvar_init!(pinned, "SlotAllocator::cond");

        // SAFETY: `inner` is pinned when `alloc` is.
        let pinned = unsafe { alloc.as_mut().map_unchecked_mut(|s| &mut s.inner) };
        kernel::mutex_init!(pinned, "SlotAllocator::inner");

        Ok(SlotAllocator(alloc.into()))
    }

    pub(crate) fn get(&self, hint: u32) -> Result<(u32, Guard<T>)> {
        let mut inner = self.0.inner.lock();

        if (hint as usize) < inner.slots.len() {
            if let Some(item) = inner.slots[hint as usize].0.take() {
                return Ok((
                    hint,
                    Guard {
                        item: Some(item),
                        slot: hint,
                        alloc: self.0.clone(),
                    },
                ));
            }
        }

        let mut first = true;
        let slot = loop {
            let mut oldest_time = u64::MAX;
            let mut oldest_slot = 0u32;

            for (i, slot) in inner.slots.iter().enumerate() {
                if slot.0.is_some() && slot.1 < oldest_time {
                    oldest_slot = i as u32;
                    oldest_time = slot.1;
                }
            }

            if oldest_time == u64::MAX {
                if first {
                    pr_warn!("{}: out of slots, blocking", core::any::type_name::<Self>());
                }
                first = false;
                if self.0.cond.wait(&mut inner) {
                    return Err(ERESTARTSYS);
                }
            } else {
                break oldest_slot;
            }
        };

        let item = inner.slots[slot as usize]
            .0
            .take()
            .expect("Someone stole our slot?");
        Ok((
            slot,
            Guard {
                item: Some(item),
                slot,
                alloc: self.0.clone(),
            },
        ))
    }
}

impl<T: SlotItem> Clone for SlotAllocator<T> {
    fn clone(&self) -> Self {
        SlotAllocator(self.0.clone())
    }
}

impl<T: SlotItem> Drop for Guard<T> {
    fn drop(&mut self) {
        let mut inner = self.alloc.inner.lock();
        if inner.slots[self.slot as usize].0.is_some() {
            pr_crit!(
                "{}: tried to return an item into a full slot ({})",
                core::any::type_name::<Self>(),
                self.slot
            );
        } else {
            inner.now += 1;
            inner.slots[self.slot as usize] = (self.item.take(), inner.now);
            self.alloc.cond.notify_one();
        }
    }
}
