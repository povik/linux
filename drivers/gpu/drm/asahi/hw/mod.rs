// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Hardware configuration

use crate::fw::types::*;
use alloc::vec::Vec;

pub(crate) struct PState {
    pub(crate) voltage: u32,
    pub(crate) frequency: u32,
    pub(crate) max_power: u32,
}

pub(crate) struct IOMapping {
    pub(crate) base: usize,
    pub(crate) size: usize,
    pub(crate) range_size: usize,
    pub(crate) writable: bool,
}

impl IOMapping {
    pub(crate) const fn new(
        base: usize,
        size: usize,
        range_size: usize,
        writable: bool,
    ) -> IOMapping {
        IOMapping {
            base,
            size,
            range_size,
            writable,
        }
    }
}

pub(crate) struct HwConfig {
    pub(crate) chip_id: u32,
    pub(crate) min_volt: u32,
    pub(crate) k: F32,
    pub(crate) io_mappings: &'static [Option<IOMapping>],
}

pub(crate) struct HwDynConfig {
    pub(crate) uat_ttb_base: u64,
    pub(crate) perf_states: Vec<PState>,
}

pub(crate) mod t8103;
