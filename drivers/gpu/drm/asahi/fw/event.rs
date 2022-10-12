// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! GPU events & stamps

use super::types::*;
use crate::trivial_gpustruct;
use core::sync::atomic::Ordering;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Clone, Copy, Default)]
    #[repr(C)]
    pub(crate) struct LinkedListHead {
        pub(crate) prev: Option<GpuWeakPointer<LinkedListHead>>,
        pub(crate) next: Option<GpuWeakPointer<LinkedListHead>>,
    }

    #[derive(Debug, Clone, Copy, Default)]
    #[repr(C)]
    pub(crate) struct NotifierList {
        pub(crate) list_head: LinkedListHead,
        pub(crate) unkptr_10: U64,
    }

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct NotifierState {
        unk_14: u32,
        unk_18: U64,
        unk_20: u32,
        unk_24: u32,
        has_vtx: u32,
        pstamp_vtx: U64,
        unk_vtx: Array<0x18, u8>,
        has_frag: u32,
        pstamp_frag: U64,
        unk_frag: Array<0x18, u8>,
        has_comp: u32,
        pstamp_comp: U64,
        unk_comp: Array<0x18, u8>,
        in_list: u32,
        list_head: LinkedListHead,
        unk_buf: Array<0x8, u8>, // Init to all-ff
    }

    impl Default for NotifierState {
        fn default() -> Self {
            let mut s: Self = unsafe { core::mem::zeroed() };
            s.unk_buf = Array::new([0xff; 0x8]);
            s
        }
    }

    #[derive(Debug, Default)]
    pub(crate) struct Threshold(AtomicU64);

    impl Threshold {
        pub(crate) fn increment(&self) {
            self.0.fetch_add(1, Ordering::Release);
        }
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct Notifier<'a> {
        pub(crate) threshold: GpuPointer<'a, super::Threshold>,
        pub(crate) generation: AtomicU32,
        pub(crate) cur_count: AtomicU32,
        pub(crate) unk_10: AtomicU32,
        pub(crate) state: NotifierState,
    }
}

trivial_gpustruct!(Threshold);
trivial_gpustruct!(NotifierList);

#[derive(Debug)]
pub(crate) struct Notifier {
    pub(crate) threshold: GpuObject<Threshold>,
}

impl GpuStruct for Notifier {
    type Raw<'a> = raw::Notifier<'a>;
}
