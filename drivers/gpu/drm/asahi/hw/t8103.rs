// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Hardware configuration for t8103 platforms

use crate::const_f32;

use super::*;

pub(crate) const HWCONFIG: super::HwConfig = HwConfig {
    chip_id: 0x8103,
    min_volt: 850,
    k: const_f32!(1.02),
    io_mappings: &[
        Some(IOMapping::new(0x204d00000, 0x1c000, 0x1c000, true)), // Fender
        Some(IOMapping::new(0x20e100000, 0x4000, 0x4000, false)),  // AICTimer
        Some(IOMapping::new(0x23b104000, 0x4000, 0x4000, true)),   // AICSWInt
        Some(IOMapping::new(0x204000000, 0x20000, 0x20000, true)), // RGX
        None,                                                      // UVD
        None,                                                      // unused
        None,                                                      // DisplayUnderrunWA
        Some(IOMapping::new(0x23b2e8000, 0x1000, 0x1000, false)),  // AnalogTempSensorControllerRegs
        Some(IOMapping::new(0x23bc00000, 0x1000, 0x1000, true)),   // PMPDoorbell
        Some(IOMapping::new(0x204d80000, 0x5000, 0x5000, true)),   // MetrologySensorRegs
        Some(IOMapping::new(0x204d61000, 0x1000, 0x1000, true)),   // GMGIFAFRegs
        Some(IOMapping::new(0x200000000, 0xd6400, 0xd6400, true)), // MCache registers
        None,                                                      // AICBankedRegisters
        Some(IOMapping::new(0x23b738000, 0x1000, 0x1000, true)),   // PMGRScratch
        None, // NIA Special agent idle register die 0
        None, // NIA Special agent idle register die 1
        None, // CRE registers
        None, // Streaming codec registers
        None, //
        None, //
    ],
};
