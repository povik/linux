// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Asahi GPU work queues

use crate::fw::channels::{PipeType, RunWorkQueueMsg};
use crate::fw::event::NotifierList;
use crate::fw::types::*;
use crate::fw::workqueue::*;
use crate::{alloc, channel, event, gpu, object};
use crate::{box_in_place, inner_ptr, place};
use core::mem;
use core::sync::atomic::Ordering;
use core::time::Duration;
use kernel::{
    dbg,
    prelude::*,
    sync::{Arc, CondVar, Guard, Mutex, UniqueArc},
};

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct WorkToken(u64);

struct WorkQueueInner {
    event_manager: Arc<event::EventManager>,
    info: GpuObject<QueueInfo>,
    new: bool,
    pipe_type: PipeType,
    size: u32,
    wptr: u32,
    pending: Vec<Box<dyn object::OpaqueGpuObject>>,
    batches: Vec<(event::EventValue, usize)>,
    last_token: Option<event::Token>,
    event: Option<(event::Event, event::EventValue)>,
}

pub(crate) struct WorkQueue {
    inner: Mutex<WorkQueueInner>,
    cond: CondVar,
}

const WQ_SIZE: u32 = 0x500;

impl WorkQueueInner {
    fn doneptr(&self) -> u32 {
        self.info
            .state
            .with(|raw, _inner| raw.gpu_doneptr.load(Ordering::Acquire))
    }
}

pub(crate) struct WorkQueueBatch<'a> {
    queue: &'a WorkQueue,
    inner: Guard<'a, Mutex<WorkQueueInner>>,
    commands: usize,
}

impl WorkQueue {
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
        event_manager: Arc<event::EventManager>,
        pipe_type: PipeType,
    ) -> Result<Arc<WorkQueue>> {
        let mut info = box_in_place!(QueueInfo {
            state: alloc.shared.new_default::<RingState>()?,
            ring: alloc.shared.array_empty(WQ_SIZE as usize)?,
            notifier_list: alloc.private.new_default::<NotifierList>()?,
            gpu_buf: alloc.private.array_empty(0x2c18)?,
            gpu_context: alloc.shared.new_default::<GpuContextData>()?,
        })?;

        let self_ptr = info.notifier_list.weak_pointer();
        info.notifier_list.with_mut(|raw, _inner| {
            raw.list_head.next = Some(inner_ptr!(&self_ptr, list_head));
        });

        info.state.with_mut(|raw, _inner| {
            raw.rb_size = WQ_SIZE;
        });

        let inner = WorkQueueInner {
            event_manager,
            info: alloc.private.new_boxed(info, |inner, ptr| {
                Ok(place!(
                    ptr,
                    raw::QueueInfo {
                        state: inner.state.gpu_pointer(),
                        ring: inner.ring.gpu_pointer(),
                        notifier_list: inner.notifier_list.gpu_pointer(),
                        gpu_buf: inner.gpu_buf.gpu_pointer(),
                        gpu_rptr1: Default::default(),
                        gpu_rptr2: Default::default(),
                        gpu_rptr3: Default::default(),
                        event_id: AtomicI32::new(-1),
                        priority: Default::default(),
                        unk_4c: -1,
                        uuid: 0xdeadbeef,
                        unk_54: -1,
                        unk_58: Default::default(),
                        busy: Default::default(),
                        __pad: Default::default(),
                        unk_84_state: Default::default(),
                        unk_88: Default::default(),
                        unk_8c: Default::default(),
                        unk_90: Default::default(),
                        unk_94: Default::default(),
                        pending: Default::default(),
                        unk_9c: Default::default(),
                        gpu_context: inner.gpu_context.gpu_pointer(),
                        unk_a8: Default::default(),
                    }
                ))
            })?,
            new: false,
            pipe_type,
            size: WQ_SIZE,
            wptr: 0,
            pending: Vec::new(),
            batches: Vec::new(),
            last_token: None,
            event: None,
        };

        let mut queue = Pin::from(UniqueArc::try_new(Self {
            // SAFETY: `condvar_init!` is called below.
            cond: unsafe { CondVar::new() },
            // SAFETY: `mutex_init!` is called below.
            inner: unsafe { Mutex::new(inner) },
        })?);

        // SAFETY: `cond` is pinned when `queue` is.
        let pinned = unsafe { queue.as_mut().map_unchecked_mut(|s| &mut s.cond) };
        kernel::condvar_init!(pinned, "WorkQueue::cond");

        // SAFETY: `inner` is pinned when `queue` is.
        let pinned = unsafe { queue.as_mut().map_unchecked_mut(|s| &mut s.inner) };
        kernel::mutex_init!(pinned, "WorkQueue::inner");

        Ok(queue.into())
    }

    pub(crate) fn begin_batch<'a>(&'a self) -> Result<WorkQueueBatch<'a>> {
        let mut inner = self.inner.lock();

        if inner.event.is_none() {
            let event = inner.event_manager.get(inner.last_token)?;
            let cur = event.current();
            inner.last_token = Some(event.token());
            inner.event = Some((event, cur));
        }

        Ok(WorkQueueBatch {
            queue: self,
            inner,
            commands: 0,
        })
    }

    pub(crate) fn poll_complete(&self) -> bool {
        let mut inner = self.inner.lock();
        let event = inner.event.as_ref();
        let cur_value = match event {
            None => {
                pr_err!("WorkQueue: poll_complete() called but no event?");
                return true;
            }
            Some(event) => event.0.current(),
        };

        let mut completed_commands: usize = 0;
        let mut batches: usize = 0;

        for (value, commands) in inner.batches.iter() {
            if value <= &cur_value {
                completed_commands += commands;
                batches += 1;
            } else {
                break;
            }
        }

        inner.batches.drain(..batches);
        inner.pending.drain(..completed_commands);
        self.cond.notify_all();
        inner.batches.is_empty()
    }
}

impl<'a> WorkQueueBatch<'a> {
    pub(crate) fn add<T: Command>(&'a mut self, command: Box<GpuObject<T>>) -> Result {
        let inner = &mut self.inner;

        let next_wptr = (inner.wptr + 1) % inner.size;
        if inner.doneptr() == next_wptr {
            pr_err!("Work queue ring buffer is full! Waiting...");
            while inner.doneptr() == next_wptr {
                if self.queue.cond.wait(inner) {
                    return Err(ERESTARTSYS);
                }
            }
        }
        inner.pending.try_reserve(1)?;

        let wptr = inner.wptr;
        inner.info.ring[wptr as usize] = command.gpu_va().get();

        inner
            .info
            .state
            .with(|raw, _inner| raw.cpu_wptr.store(next_wptr, Ordering::Release));

        // Cannot fail, since we did a try_reserve(1) above
        inner
            .pending
            .try_push(command)
            .expect("try_push() failed after try_reserve(1)");
        self.commands += 1;
        Ok(())
    }

    pub(crate) fn commit(&mut self) -> Result<event::EventValue> {
        let inner = &mut self.inner;
        let event = inner.event.as_mut().expect("WorkQueueBatch lost its event");

        if self.commands == 0 {
            return Ok(event.1);
        }

        event.1.increment();
        let event_value = event.1;

        inner.batches.try_push((event_value, self.commands))?;
        self.commands = 0;
        Ok(event_value)
    }

    pub(crate) fn submit(mut self, channel: &mut channel::PipeChannel) -> Result {
        self.commit()?;

        let inner = &mut self.inner;
        let event = inner.event.as_ref().expect("WorkQueueBatch lost its event");
        let msg = RunWorkQueueMsg {
            pipe_type: inner.pipe_type,
            work_queue: Some(inner.info.weak_pointer()),
            wptr: inner.wptr,
            event_slot: event.0.slot(),
            is_new: inner.new,
            __pad: Default::default(),
        };
        channel.send(&msg);
        inner.new = false;
        Ok(())
    }
}
