// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Debug enable/disable flags

pub(crate) use super::{cls_dev_dbg, cls_pr_debug, debug, mod_dev_dbg, mod_pr_debug};
use core::sync::atomic::{AtomicU64, Ordering};

static DEBUG_FLAGS: AtomicU64 = AtomicU64::new(0);

pub(crate) enum DebugFlags {
    // 0-3: Memory-related debug
    Mmu = 0,
    Alloc = 1,
    Gem = 2,
    Object = 3,

    // 4-7: Slots and firmware objects
    SlotAlloc = 4,
    Event = 5,
    Buffer = 6,
    WorkQueue = 7,

    // 8-11: DRM interface, rendering, GPU globals
    File = 8,
    Render = 9,
    Gpu = 10,
    Init = 11,

    // 12-19: Channels
    FwLogCh = 12,
    KTraceCh = 13,
    StatsCh = 14,
    EventCh = 15,
    PipeCh = 16,
    DeviceControlCh = 17,
    FwCtlCh = 18,

    // 32-: Behavior flags
    ConservativeTlbi = 32,
    KeepGpuPowered = 33,
    WaitForPowerOff = 34,
    NoGpuRecovery = 35,
}

pub(crate) fn update_debug_flags() {
    let flags = {
        let lock = crate::THIS_MODULE.kernel_param_lock();
        *crate::debug_flags.read(&lock)
    };

    DEBUG_FLAGS.store(flags, Ordering::Relaxed);
}

#[inline(always)]
pub(crate) fn debug_enabled(flag: DebugFlags) -> bool {
    DEBUG_FLAGS.load(Ordering::Relaxed) & 1 << (flag as usize) != 0
}

#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {
        if $crate::debug::debug_enabled(DEBUG_CLASS) {
            $($arg)*
        }
    };
}

#[macro_export]
macro_rules! mod_pr_debug (
    ($($arg:tt)*) => (
        $crate::debug! { ::kernel::pr_info! ( $($arg)* ); }
    )
);

#[macro_export]
macro_rules! mod_dev_dbg (
    ($($arg:tt)*) => (
        $crate::debug! { ::kernel::dev_info! ( $($arg)* ); }
    )
);

#[macro_export]
macro_rules! cls_pr_debug (
    ($cls:ident, $($arg:tt)*) => (
        if $crate::debug::debug_enabled($crate::debug::DebugFlags::$cls) {
            ::kernel::pr_info! ( $($arg)* );
        }
    )
);

#[macro_export]
macro_rules! cls_dev_dbg (
    ($cls:ident, $($arg:tt)*) => (
        if $crate::debug::debug_enabled($crate::debug::DebugFlags::$cls) {
            ::kernel::dev_info! ( $($arg)* );
        }
    )
);
