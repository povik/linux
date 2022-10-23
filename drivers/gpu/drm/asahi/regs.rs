// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

use kernel::{io_mem::IoMem, platform, prelude::*};

pub(crate) const ASC_CTL_SIZE: usize = 0x4000;
pub(crate) const SGX_SIZE: usize = 0x1000000;

const CPU_CONTROL: usize = 0x44;
const CPU_RUN: u32 = 0x1 << 4; // BIT(4)

const FAULT_INFO: usize = 0x17030;

#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug)]
pub(crate) enum FaultUnit {
    DCMP(u8),
    UL1C(u8),
    CMP(u8),
    GSL1(u8),
    IAP(u8),
    VCE(u8),
    TE(u8),
    RAS(u8),
    VDM(u8),
    PPP(u8),
    IPF(u8),
    IPF_CPF(u8),
    VF(u8),
    VF_CPF(u8),
    ZL(u8),

    dPM,
    dCDM_KS(u8),
    dIPP,
    dIPP_CS,
    dVDM_CSD,
    dVDM_SSD,
    dVDM_ILF,
    dVDM_ILD,
    dRDE(u8),
    FC,
    GSL2,

    GL2CC_META(u8),
    GL2CC_MB,

    gPM_SP(u8),
    gVDM_CSD_SP(u8),
    gVDM_SSD_SP(u8),
    gVDM_ILF_SP(u8),
    gVDM_TFP_SP(u8),
    gVDM_MMB_SP(u8),
    gCDM_CS_KS0_SP(u8),
    gCDM_CS_KS1_SP(u8),
    gCDM_CS_KS2_SP(u8),
    gCDM_KS0_SP(u8),
    gCDM_KS1_SP(u8),
    gCDM_KS2_SP(u8),
    gIPP_SP(u8),
    gIPP_CS_SP(u8),
    gRDE0_SP(u8),
    gRDE1_SP(u8),

    Unknown(u8),
}

#[derive(Copy, Clone, Debug)]
pub(crate) enum FaultReason {
    Unmapped,
    AfFault,
    WriteOnly,
    ReadOnly,
    NoAccess,
    Unknown(u8),
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct FaultInfo {
    address: u64,
    write: bool,
    vm_slot: u32,
    unit: FaultUnit,
    unk_8: bool,
    reason: FaultReason,
}

pub(crate) struct Resources {
    asc: IoMem<ASC_CTL_SIZE>,
    sgx: IoMem<SGX_SIZE>,
}

impl Resources {
    pub(crate) fn new(pdev: &mut platform::Device) -> Result<Resources> {
        // TODO: add device abstraction to ioremap by name
        let asc_res = unsafe { pdev.ioremap_resource(0)? };
        let sgx_res = unsafe { pdev.ioremap_resource(1)? };

        Ok(Resources {
            // SAFETY: This device does DMA via the UAT IOMMU.
            asc: asc_res,
            sgx: sgx_res,
        })
    }

    fn sgx_read32(&self, off: usize) -> u32 {
        self.sgx.readl_relaxed(off)
    }

    fn sgx_write32(&self, off: usize, val: u32) {
        self.sgx.writel_relaxed(val, off)
    }

    fn sgx_read64(&self, off: usize) -> u64 {
        self.sgx.readq_relaxed(off)
    }

    fn sgx_write64(&self, off: usize, val: u64) {
        self.sgx.writeq_relaxed(val, off)
    }

    pub(crate) fn init_mmio(&self) -> Result {
        // Nothing to do for now...

        Ok(())
    }

    pub(crate) fn start_cpu(&self) -> Result {
        let val = self.asc.readl_relaxed(CPU_CONTROL);

        self.asc.writel_relaxed(val | CPU_RUN, CPU_CONTROL);

        Ok(())
    }

    pub(crate) fn get_fault_info(&self) -> Option<FaultInfo> {
        let fault_info = self.sgx_read64(FAULT_INFO);

        if fault_info & 1 == 0 {
            return None;
        }

        let unit_code = ((fault_info >> 9) & 0xff) as u8;
        let unit = match unit_code {
            0x00..=0x9f => match unit_code & 0xf {
                0x0 => FaultUnit::DCMP(unit_code >> 4),
                0x1 => FaultUnit::UL1C(unit_code >> 4),
                0x2 => FaultUnit::CMP(unit_code >> 4),
                0x3 => FaultUnit::GSL1(unit_code >> 4),
                0x4 => FaultUnit::IAP(unit_code >> 4),
                0x5 => FaultUnit::VCE(unit_code >> 4),
                0x6 => FaultUnit::TE(unit_code >> 4),
                0x7 => FaultUnit::RAS(unit_code >> 4),
                0x8 => FaultUnit::VDM(unit_code >> 4),
                0x9 => FaultUnit::PPP(unit_code >> 4),
                0xa => FaultUnit::IPF(unit_code >> 4),
                0xb => FaultUnit::IPF_CPF(unit_code >> 4),
                0xc => FaultUnit::VF(unit_code >> 4),
                0xd => FaultUnit::VF_CPF(unit_code >> 4),
                0xe => FaultUnit::VF_CPF(unit_code >> 4),
                _ => FaultUnit::Unknown(unit_code),
            },
            0xa1 => FaultUnit::dPM,
            0xa2 => FaultUnit::dCDM_KS(0),
            0xa3 => FaultUnit::dCDM_KS(1),
            0xa4 => FaultUnit::dCDM_KS(2),
            0xa5 => FaultUnit::dIPP,
            0xa6 => FaultUnit::dIPP_CS,
            0xa7 => FaultUnit::dVDM_CSD,
            0xa8 => FaultUnit::dVDM_SSD,
            0xa9 => FaultUnit::dVDM_ILF,
            0xaa => FaultUnit::dVDM_ILD,
            0xab => FaultUnit::dRDE(0),
            0xac => FaultUnit::dRDE(1),
            0xad => FaultUnit::FC,
            0xae => FaultUnit::GSL2,
            0xb0..=0xb7 => FaultUnit::GL2CC_META((unit_code & 0xf) as u8),
            0xb8 => FaultUnit::GL2CC_MB,
            0xe0..=0xff => match unit_code & 0xf {
                0x0 => FaultUnit::gPM_SP((unit_code >> 4) & 1),
                0x1 => FaultUnit::gVDM_CSD_SP((unit_code >> 4) & 1),
                0x2 => FaultUnit::gVDM_SSD_SP((unit_code >> 4) & 1),
                0x3 => FaultUnit::gVDM_ILF_SP((unit_code >> 4) & 1),
                0x4 => FaultUnit::gVDM_TFP_SP((unit_code >> 4) & 1),
                0x5 => FaultUnit::gVDM_MMB_SP((unit_code >> 4) & 1),
                0x6 => FaultUnit::gCDM_CS_KS0_SP((unit_code >> 4) & 1),
                0x7 => FaultUnit::gCDM_CS_KS1_SP((unit_code >> 4) & 1),
                0x8 => FaultUnit::gCDM_CS_KS2_SP((unit_code >> 4) & 1),
                0x9 => FaultUnit::gCDM_KS0_SP((unit_code >> 4) & 1),
                0xa => FaultUnit::gCDM_KS1_SP((unit_code >> 4) & 1),
                0xb => FaultUnit::gCDM_KS2_SP((unit_code >> 4) & 1),
                0xc => FaultUnit::gIPP_SP((unit_code >> 4) & 1),
                0xd => FaultUnit::gIPP_CS_SP((unit_code >> 4) & 1),
                0xe => FaultUnit::gRDE0_SP((unit_code >> 4) & 1),
                0xf => FaultUnit::gRDE1_SP((unit_code >> 4) & 1),
                _ => FaultUnit::Unknown(unit_code),
            },
            _ => FaultUnit::Unknown(unit_code),
        };

        let reason = match (fault_info >> 1) & 0x7 {
            0 => FaultReason::Unmapped,
            1 => FaultReason::AfFault,
            2 => FaultReason::WriteOnly,
            3 => FaultReason::ReadOnly,
            4 => FaultReason::NoAccess,
            a => FaultReason::Unknown(a as u8),
        };

        Some(FaultInfo {
            address: fault_info >> 24,
            write: fault_info & (1 << 23) != 0,
            vm_slot: ((fault_info >> 17) & 0x3f) as u32,
            unit,
            unk_8: fault_info & (1 << 8) != 0,
            reason,
        })
    }
}
