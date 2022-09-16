// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]

//! Asahi GPU work queues

use crate::fw::channels::{PipeType, RunWorkQueueMsg};
use crate::fw::event::NotifierList;
use crate::fw::types::*;
use crate::fw::workqueue::*;
use crate::{alloc, channel, gpu, object};
use crate::{box_in_place, inner_ptr, place};
use core::sync::atomic::Ordering;
use core::time::Duration;
use kernel::{
    dbg,
    prelude::*,
    sync::{Arc, CondVar, Mutex, UniqueArc},
};

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct WorkToken(u64);

struct WorkQueueInner {
    info: GpuObject<QueueInfo>,
    new: bool,
    pipe_type: PipeType,
    size: u32,
    wptr: u32,
    cur_token: WorkToken,
    oldest_token: WorkToken,
    pending: Vec<Box<dyn object::OpaqueGpuObject>>,
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

impl WorkQueue {
    pub(crate) fn new(
        alloc: &mut gpu::KernelAllocators,
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
            cur_token: Default::default(),
            oldest_token: Default::default(),
            pending: Vec::new(),
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

    pub(crate) fn submit<T: Command>(&self, command: Box<GpuObject<T>>) -> Result<WorkToken> {
        let mut inner = self.inner.lock();

        let next_wptr = (inner.wptr + 1) % inner.size;
        if inner.doneptr() == next_wptr {
            pr_err!("Work queue ring buffer is full! Waiting...");
            while inner.doneptr() == next_wptr {
                if self.cond.wait(&mut inner) {
                    return Err(ERESTARTSYS);
                }
            }
        }
        inner.pending.try_reserve(1)?;

        let wptr = inner.wptr;
        inner.info.ring.as_mut_slice()[wptr as usize] = command.gpu_va().get();

        inner
            .info
            .state
            .with(|raw, _inner| raw.cpu_wptr.store(next_wptr, Ordering::Release));

        // Cannot fail, since we did a try_reserve(1) above
        inner
            .pending
            .try_push(command)
            .expect("try_push() failed after try_reserve(1)");
        inner.cur_token = WorkToken(inner.cur_token.0 + 1);
        Ok(inner.cur_token)
    }

    pub(crate) fn run(&self, channel: &mut channel::PipeChannel, stamp_index: u32) -> Result {
        let mut inner = self.inner.lock();

        let msg = RunWorkQueueMsg {
            pipe_type: inner.pipe_type,
            work_queue: inner.info.weak_pointer(),
            wptr: inner.wptr,
            stamp_index,
            is_new: inner.new,
            __pad: Default::default(),
        };
        channel.send(&msg);
        inner.new = false;
        Ok(())
    }

    pub(crate) fn complete_until(&self, token: WorkToken) -> Result {
        let mut inner = self.inner.lock();

        if token == inner.oldest_token {
            Ok(())
        } else if token < inner.oldest_token || token > inner.cur_token {
            pr_err!(
                "WorkQueue: completion token {:?} out of order ({:?}..{:?})",
                token,
                inner.oldest_token,
                inner.cur_token
            );
            Err(EINVAL)
        } else {
            let count = (token.0 - inner.oldest_token.0) as usize;
            // TODO: .drain() has bad performance, maybe get VecDeque into the kernel?
            inner.pending.drain(..count);
            inner.oldest_token = token;
            self.cond.notify_all();
            Ok(())
        }
    }
}
