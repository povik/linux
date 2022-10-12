// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! GPU initialization / global structures

use super::channels;
use super::types::*;
use crate::no_debug;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct ChannelRing<T: GpuStruct + Debug + Default, U: Copy> {
        pub(crate) state: Option<GpuWeakPointer<T>>,
        pub(crate) ring: Option<GpuWeakPointer<[U]>>,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct PipeChannels {
        pub(crate) vtx: ChannelRing<channels::ChannelState, channels::PipeMsg>,
        pub(crate) frag: ChannelRing<channels::ChannelState, channels::PipeMsg>,
        pub(crate) comp: ChannelRing<channels::ChannelState, channels::PipeMsg>,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct FwStatusFlags {
        pub(crate) halt_count: AtomicU32,
        __pad0: Pad<0xc>,
        pub(crate) halted: AtomicU32,
        __pad1: Pad<0xc>,
        pub(crate) resume: AtomicU32,
        __pad2: Pad<0xc>,
        pub(crate) unk_40: u32,
        __pad3: Pad<0xc>,
        pub(crate) unk_ctr: u32,
        __pad4: Pad<0xc>,
        pub(crate) unk_60: u32,
        __pad5: Pad<0xc>,
        pub(crate) unk_70: u32,
        __pad6: Pad<0xc>,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct FwStatus {
        pub(crate) fwctl_channel: ChannelRing<channels::FwCtlChannelState, channels::FwCtlMsg>,
        pub(crate) flags: FwStatusFlags,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct HwDataShared1 {
        pub(crate) unk_0: u32,
        pub(crate) unk_4: u32,
        pub(crate) unk_8: u32,
        pub(crate) unk_c: u32,
        pub(crate) unk_10: u32,
        pub(crate) unk_14: u32,
        pub(crate) unk_18: u32,
        pub(crate) unk_1c: u32,
        pub(crate) unk_20: Array<0x26, u8>,
        pub(crate) unk_46: Array<0x6, u8>,
        pub(crate) unk_4c: Array<0x58, u8>,
        pub(crate) unk_a4: u32,
        pub(crate) unk_a8: u32,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct HwDataShared2 {
        pub(crate) unk_ac: u32,
        pub(crate) unk_b0: u32,
        pub(crate) unk_b4: Array<0x18, u8>,
        pub(crate) unk_cc: Array<0x8, u8>,
        pub(crate) unk_d4: Array<0x10, u8>,
        pub(crate) unk_e4: Array<0x8, u8>,
        pub(crate) unk_ec: Array<0x4c8, u8>,
        pub(crate) unk_5b4: u32,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct HwDataA130Extra {
        pub(crate) unk_0: Array<0x38, u8>,
        pub(crate) unk_38: u32,
        pub(crate) unk_3c: u32,
        pub(crate) unk_40: u32,
        pub(crate) unk_44: u32,
        pub(crate) unk_48: u32,
        pub(crate) unk_4c: u32,
        pub(crate) unk_50: u32,
        pub(crate) unk_54: u32,
        pub(crate) unk_58: u32,
        pub(crate) unk_5c: u32,
        pub(crate) unk_60: F32,
        pub(crate) unk_64: F32,
        pub(crate) unk_68: F32,
        pub(crate) unk_6c: F32,
        pub(crate) unk_70: F32,
        pub(crate) unk_74: F32,
        pub(crate) unk_78: F32,
        pub(crate) unk_7c: F32,
        pub(crate) unk_80: F32,
        pub(crate) unk_84: F32,
        pub(crate) unk_88: u32,
        pub(crate) unk_8c: u32,
        pub(crate) unk_90: u32,
        pub(crate) unk_94: u32,
        pub(crate) unk_98: u32,
        pub(crate) unk_9c: F32,
        pub(crate) unk_a0: u32,
        pub(crate) unk_a4: u32,
        pub(crate) unk_a8: u32,
        pub(crate) unk_ac: u32,
        pub(crate) unk_b0: u32,
        pub(crate) unk_b4: u32,
        pub(crate) unk_b8: u32,
        pub(crate) unk_bc: u32,
        pub(crate) unk_c0: u32,
        pub(crate) unk_c4: F32,
        pub(crate) unk_c8: Array<0x4c, u8>,
        pub(crate) unk_114: F32,
        pub(crate) unk_118: u32,
        pub(crate) unk_11c: u32,
        pub(crate) unk_120: u32,
        pub(crate) unk_124: u32,
        pub(crate) unk_128: u32,
        pub(crate) unk_12c: Array<0x8c, u8>,
    }

    #[versions(AGX)]
    #[derive(Default)]
    #[repr(C)]
    pub(crate) struct HwDataA {
        pub(crate) unk_0: u32,
        pub(crate) unk_4: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_8_0: u32,

        pub(crate) unk_8: u32,
        pub(crate) pwr_status: AtomicU32,
        pub(crate) unk_10: F32,
        pub(crate) unk_14: u32,
        pub(crate) unk_18: u32,
        pub(crate) unk_1c: u32,
        pub(crate) unk_20: u32,
        pub(crate) unk_24: u32,
        pub(crate) actual_pstate: u32,
        pub(crate) tgt_pstate: u32,
        pub(crate) unk_30: u32,
        pub(crate) cur_pstate: u32,
        pub(crate) unk_38: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_3c_0: u32,

        pub(crate) unk_3c: u32,
        pub(crate) unk_40: u32,
        pub(crate) unk_44: u32,
        pub(crate) unk_48: u32,
        pub(crate) unk_4c: u32,
        pub(crate) freq_mhz: F32,
        pub(crate) unk_54: Array<0x20, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_74_0: u32,

        pub(crate) unk_74: Array<0x10, F32>,
        pub(crate) unk_b4: Array<0x100, u8>,
        pub(crate) unk_1b4: u32,
        pub(crate) temp_c: u32,
        pub(crate) avg_power_mw: u32,
        pub(crate) update_ts: U64,
        pub(crate) unk_1c8: u32,
        pub(crate) unk_1cc: Array<0x478, u8>,
        pub(crate) pad_644: Pad<0x8>,
        pub(crate) unk_64c: u32,
        pub(crate) unk_650: u32,
        pub(crate) pad_654: u32,
        pub(crate) unk_658: F32,
        pub(crate) pad_65c: u32,
        pub(crate) unk_660: F32,
        pub(crate) pad_664: u32,
        pub(crate) unk_668: F32,
        pub(crate) pad_66c: u32,
        pub(crate) unk_670: u32,
        pub(crate) unk_674: F32,
        pub(crate) unk_678: F32,
        pub(crate) pad_67c: u32,
        pub(crate) unk_680: u32,
        pub(crate) unk_684: u32,
        pub(crate) unk_688: u32,
        pub(crate) unk_68c: u32,
        pub(crate) pad_690: u32,
        pub(crate) unk_694: u32,
        pub(crate) unk_698: u32,
        pub(crate) pad_69c: Pad<0x18>,
        pub(crate) unk_6b4: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_6b8_0: Array<0x10, u8>,

        pub(crate) unk_6b8: u32,
        pub(crate) unk_6bc: u32,
        pub(crate) pad_6c0: Pad<0x14>,
        pub(crate) unk_6d4: u32,
        pub(crate) unk_6d8: u32,
        pub(crate) pad_6dc: u32,
        pub(crate) unk_6e0: F32,
        pub(crate) pad_6e4: u32,
        pub(crate) unk_6e8: F32,
        pub(crate) pad_6ec: u32,
        pub(crate) unk_6f0: F32,
        pub(crate) pad_6f4: u32,
        pub(crate) unk_6f8: u32,
        pub(crate) unk_6fc: F32,
        pub(crate) unk_700: F32,
        pub(crate) pad_704: u32,
        pub(crate) unk_708: u32,
        pub(crate) unk_70c: u32,
        pub(crate) unk_710: u32,
        pub(crate) unk_714: u32,
        pub(crate) pad_718: u32,
        pub(crate) unk_71c: F32,
        pub(crate) unk_720: u32,
        pub(crate) cur_power_mw_2: u32,
        pub(crate) unk_728: u32,
        pub(crate) unk_72c: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_730_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_730_4: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_730_8: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_730_c: u32,

        pub(crate) unk_730: F32,
        pub(crate) unk_734: u32,
        pub(crate) unk_738: u32,
        pub(crate) unk_73c: u32,
        pub(crate) unk_740: u32,
        pub(crate) unk_744: u32,
        pub(crate) unk_748: Array<0x4, F32>,
        pub(crate) unk_758: u32,
        pub(crate) unk_75c: u32,
        pub(crate) pad_760: u32,
        pub(crate) unk_764: u32,
        pub(crate) unk_768: u32,
        pub(crate) unk_76c: u32,
        pub(crate) pad_770: u32,
        pub(crate) unk_774: u32,
        pub(crate) unk_778: u32,
        pub(crate) unk_77c: u32,
        pub(crate) unk_780: F32,
        pub(crate) unk_784: F32,
        pub(crate) unk_788: F32,
        pub(crate) unk_78c: F32,
        pub(crate) unk_790: F32,
        pub(crate) unk_794: F32,
        pub(crate) unk_798: F32,
        pub(crate) unk_79c: F32,
        pub(crate) unk_7a0: F32,
        pub(crate) unk_7a4: F32,
        pub(crate) unk_7a8: F32,
        pub(crate) unk_7ac: u32,
        pub(crate) unk_7b0: u32,
        pub(crate) unk_7b4: u32,
        pub(crate) pad_7b8: u32,
        pub(crate) use_percent: F32,
        pub(crate) unk_7c0: u32,
        pub(crate) pad_7c4: Pad<0x18>,
        pub(crate) unk_7dc: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_7e0_0: Array<0x10, u8>,

        pub(crate) unk_7e0: u32,
        pub(crate) pad_7e4: u32,
        pub(crate) unk_7e8: Array<0x14, u8>,
        pub(crate) unk_7fc: F32,
        pub(crate) unk_800: F32,
        pub(crate) unk_804: F32,
        pub(crate) unk_808: u32,
        pub(crate) pad_80c: u32,
        pub(crate) unk_810: u32,
        pub(crate) pad_814: u32,
        pub(crate) unk_818: u32,
        pub(crate) unk_81c: u32,
        pub(crate) pad_820: u32,
        pub(crate) unk_824: F32,
        pub(crate) unk_828: u32,
        pub(crate) unk_82c: u32,
        pub(crate) unk_830: F32,
        pub(crate) unk_834: F32,
        pub(crate) unk_838: u32,
        pub(crate) unk_83c: u32,
        pub(crate) pad_840: Pad<0x2c>,
        pub(crate) unk_86c: u32,
        pub(crate) unk_870: u32,
        pub(crate) unk_874: u32,
        pub(crate) unk_878: u32,
        pub(crate) unk_87c: u32,
        pub(crate) unk_880: u32,
        pub(crate) unk_884: u32,
        pub(crate) pad_888: u32,
        pub(crate) unk_88c: u32,
        pub(crate) pad_890: u32,
        pub(crate) unk_894: F32,
        pub(crate) pad_898: u32,
        pub(crate) unk_89c: F32,
        pub(crate) pad_8a0: u32,
        pub(crate) unk_8a4: u32,
        pub(crate) unk_8a8: F32,
        pub(crate) unk_8ac: F32,
        pub(crate) pad_8b0: u32,
        pub(crate) unk_8b4: u32,
        pub(crate) unk_8b8: u32,
        pub(crate) unk_8bc: u32,
        pub(crate) unk_8c0: u32,
        pub(crate) unk_8c4: u32,
        pub(crate) unk_8c8: u32,
        pub(crate) unk_8cc: u32,
        pub(crate) pad_8d0: Pad<0x14>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_8e4_0: Array<0x10, u8>,

        pub(crate) unk_8e4: u32,
        pub(crate) unk_8e8: u32,
        pub(crate) unk_8ec: u32,
        pub(crate) unk_8f0: u32,
        pub(crate) unk_8f4: u32,
        pub(crate) pad_8f8: u32,
        pub(crate) pad_8fc: u32,
        pub(crate) unk_900: Array<0x294, u8>,
        pub(crate) unk_b94: u32,
        pub(crate) freq_with_off: u32,
        pub(crate) unk_b9c: u32,
        pub(crate) unk_ba0: u64,
        pub(crate) unk_ba8: u64,
        pub(crate) unk_bb0: u32,
        pub(crate) unk_bb4: u32,
        pub(crate) pad_bb8: Pad<0x74>,
        pub(crate) unk_c2c: u32,
        pub(crate) unk_c30: u32,
        pub(crate) unk_c34: u32,
        pub(crate) unk_c38: u32,
        pub(crate) unk_c3c: u32,
        pub(crate) unk_c40: u32,
        pub(crate) unk_c44: F32,
        pub(crate) unk_c48: F32,
        pub(crate) unk_c4c: F32,
        pub(crate) unk_c50: u32,
        pub(crate) unk_c54: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_c58_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_c58_4: u32,

        pub(crate) unk_c58: F32,
        pub(crate) unk_c5c: u32,
        pub(crate) unk_c60: u32,
        pub(crate) unk_c64: u32,
        pub(crate) unk_c68: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_c6c_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_c6c_4: u32,

        pub(crate) unk_c6c: F32,
        pub(crate) unk_c70: F32,
        pub(crate) pad_c74: u32,
        pub(crate) unk_c78: u32,
        pub(crate) unk_c7c: u32,
        pub(crate) unk_c80: u32,
        pub(crate) unk_c84: u32,
        pub(crate) unk_c88: u32,
        pub(crate) unk_c8c: u32,
        pub(crate) unk_c90: Array<0x60, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_cf0_0: Array<0x20, u8>,

        pub(crate) unk_cf0: u32,
        pub(crate) unk_cf4: u32,
        pub(crate) unk_cf8: u32,
        pub(crate) unk_cfc: u32,
        pub(crate) unk_d00: u32,
        pub(crate) unk_d04: F32,
        pub(crate) unk_d08: u32,
        pub(crate) unk_d0c: F32,
        pub(crate) unk_d10: u32,
        pub(crate) unk_d14: F32,
        pub(crate) unk_d18: u32,
        pub(crate) unk_d1c: u32,
        pub(crate) unk_d20: F32,
        pub(crate) unk_d24: F32,
        pub(crate) unk_d28: u32,
        pub(crate) unk_d2c: u32,
        pub(crate) unk_d30: u32,
        pub(crate) unk_d34: u32,
        pub(crate) unk_d38: u32,
        pub(crate) unk_d3c: u32,
        pub(crate) unk_d40: F32,
        pub(crate) unk_d44: u32,
        pub(crate) unk_d48: u32,
        pub(crate) unk_d4c: u32,
        pub(crate) unk_d50: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_d54_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_d54_4: Array<0xc, u8>,

        pub(crate) unk_d54: Array<0x10, u8>,
        pub(crate) unk_d64: u32,
        pub(crate) unk_d68: Array<0x24, u8>,
        pub(crate) unk_d8c: u32,
        pub(crate) unk_d90: u32,
        pub(crate) unk_d94: u32,
        pub(crate) unk_d98: u32,
        pub(crate) unk_d9c: F32,
        pub(crate) unk_da0: u32,
        pub(crate) unk_da4: F32,
        pub(crate) unk_da8: u32,
        pub(crate) unk_dac: F32,
        pub(crate) unk_db0: u32,
        pub(crate) unk_db4: u32,
        pub(crate) unk_db8: F32,
        pub(crate) unk_dbc: F32,
        pub(crate) unk_dc0: u32,
        pub(crate) unk_dc4: u32,
        pub(crate) unk_dc8: u32,
        pub(crate) unk_dcc: u32,
        pub(crate) unk_dd0: Array<0x40, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_e10_0: HwDataA130Extra,

        pub(crate) unk_e10: Array<0x20, u8>,
        pub(crate) pad_e30: Pad<0x7e0>,
        pub(crate) unk_1610: Array<0x28, u8>,

        #[ver(V < V13_0B4)]
        pub(crate) unk_1638: Array<0x8, u8>,

        pub(crate) unk_1640: Array<0x2000, u8>,
        pub(crate) unk_3640: u32,
        pub(crate) hws1: HwDataShared1,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_pad1: Pad<0x20>,

        pub(crate) hws2: HwDataShared2,
        pub(crate) unk_3bfc: u32,
        pub(crate) unk_3c00: Array<0xa0, u8>,
        pub(crate) unk_3ca0: u64,
        pub(crate) unk_3ca8: u64,
        pub(crate) unk_3cb0: u64,
        pub(crate) ts_last_idle: u64,
        pub(crate) ts_last_poweron: u64,
        pub(crate) ts_last_poweroff: u64,
        pub(crate) unk_3cd0: u64,
        pub(crate) unk_3cd8: u64,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_3ce0_0: u32,

        pub(crate) unk_3ce0: Array<0x40, u8>,
        pub(crate) unk_3d20: Array<0x4c, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_3d6c: Array<0x38, u8>,
    }

    #[versions(AGX)]
    no_debug!(HwDataA::ver);

    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct IOMapping {
        pub(crate) phys_addr: u64,
        pub(crate) virt_addr: u64,
        pub(crate) size: u32,
        pub(crate) range_size: u32,
        pub(crate) readwrite: u64,
    }

    #[versions(AGX)]
    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct HwDataB {
        #[ver(V < V13_0B4)]
        pub(crate) unk_0: u64,

        pub(crate) unk_8: u64,

        #[ver(V < V13_0B4)]
        pub(crate) unk_10: u64,

        pub(crate) unk_18: u64,
        pub(crate) unk_20: u64,
        pub(crate) unk_28: u64,
        pub(crate) unk_30: u64,
        pub(crate) unkptr_38: u64,
        pub(crate) pad_40: Pad<0x20>,

        #[ver(V < V13_0B4)]
        pub(crate) yuv_matrices: Array<0xf, Array<3, Array<4, i16>>>,

        #[ver(V >= V13_0B4)]
        pub(crate) yuv_matrices: Array<0x3f, Array<3, Array<4, i16>>>,

        pub(crate) pad_1c8: Pad<0x8>,
        pub(crate) io_mappings: Array<0x14, IOMapping>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_450_0: Array<0x68, u8>,

        pub(crate) chip_id: u32,
        pub(crate) unk_454: u32,
        pub(crate) unk_458: u32,
        pub(crate) unk_45c: u32,
        pub(crate) unk_460: u32,
        pub(crate) unk_464: u32,
        pub(crate) unk_468: u32,
        pub(crate) unk_46c: u32,
        pub(crate) unk_470: u32,
        pub(crate) unk_474: u32,
        pub(crate) unk_478: u32,
        pub(crate) unk_47c: u32,
        pub(crate) unk_480: u32,
        pub(crate) unk_484: u32,
        pub(crate) unk_488: u32,
        pub(crate) unk_48c: u32,
        pub(crate) unk_490: u32,
        pub(crate) unk_494: u32,
        pub(crate) pad_498: Pad<0x4>,
        pub(crate) unk_49c: u32,
        pub(crate) unk_4a0: u32,
        pub(crate) unk_4a4: u32,
        pub(crate) pad_4a8: Pad<0x4>,
        pub(crate) unk_4ac: u32,
        pub(crate) pad_4b0: Pad<0x8>,
        pub(crate) unk_4b8: u32,
        pub(crate) unk_4bc: Array<0x4, u8>,
        pub(crate) unk_4c0: u32,
        pub(crate) unk_4c4: u32,
        pub(crate) unk_4c8: u32,
        pub(crate) unk_4cc: u32,
        pub(crate) unk_4d0: u32,
        pub(crate) unk_4d4: u32,
        pub(crate) unk_4d8: Array<0x4, u8>,
        pub(crate) unk_4dc: u32,
        pub(crate) unk_4e0: u64,
        pub(crate) unk_4e8: u32,
        pub(crate) unk_4ec: u32,
        pub(crate) unk_4f0: u32,
        pub(crate) unk_4f4: u32,
        pub(crate) unk_4f8: u32,
        pub(crate) unk_4fc: u32,
        pub(crate) unk_500: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_504_0: u32,

        pub(crate) unk_504: u32,
        pub(crate) unk_508: u32,
        pub(crate) unk_50c: u32,
        pub(crate) unk_510: u32,
        pub(crate) unk_514: u32,
        pub(crate) unk_518: u32,
        pub(crate) unk_51c: u32,
        pub(crate) unk_520: u32,
        pub(crate) unk_524: u32,
        pub(crate) unk_528: u32,
        pub(crate) unk_52c: u32,
        pub(crate) unk_530: u32,
        pub(crate) unk_534: u32,
        pub(crate) unk_538: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_53c_0: u32,

        pub(crate) unk_53c: u32,
        pub(crate) unk_540: u32,
        pub(crate) unk_544: u32,
        pub(crate) unk_548: u32,
        pub(crate) unk_54c: u32,
        pub(crate) unk_550: u32,
        pub(crate) unk_554: u32,
        pub(crate) uat_context_table_base: u64,
        pub(crate) unk_560: u32,
        pub(crate) unk_564: u32,
        pub(crate) unk_568: u32,
        pub(crate) max_pstate: u32,

        #[ver(V < V13_0B4)]
        pub(crate) num_pstates: u32,

        pub(crate) frequencies: Array<0x10, u32>,
        pub(crate) voltages: Array<0x10, [u32; 0x8]>,
        pub(crate) voltages_sram: Array<0x10, [u32; 0x8]>,
        pub(crate) unk_9b4: Array<0x10, F32>,
        pub(crate) unk_9f4: Array<0x10, u32>,
        pub(crate) perf_levels: Array<0x10, u32>,
        pub(crate) unk_a74: u32,
        pub(crate) unk_a78: u32,
        pub(crate) unk_a7c: u32,
        pub(crate) unk_a80: u32,
        pub(crate) unk_a84: u32,
        pub(crate) unk_a88: u32,
        pub(crate) unk_a8c: u32,
        pub(crate) pad_a90: Pad<0x24>,

        #[ver(V < V13_0B4)]
        pub(crate) min_volt: u32,

        #[ver(V < V13_0B4)]
        pub(crate) unk_ab8: u32,

        #[ver(V < V13_0B4)]
        pub(crate) unk_abc: u32,

        #[ver(V < V13_0B4)]
        pub(crate) unk_ac0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_ac4_0: Array<0x1f0, u8>,

        pub(crate) pad_ac4: Pad<0x8>,
        pub(crate) unk_acc: u32,
        pub(crate) unk_ad0: u32,
        pub(crate) pad_ad4: Pad<0x10>,
        pub(crate) unk_ae4: Array<0x4, u32>,
        pub(crate) pad_af4: Pad<0x4>,
        pub(crate) unk_af8: u32,
        pub(crate) pad_afc: Pad<0x8>,
        pub(crate) unk_b04: u32,
        pub(crate) unk_b08: u32,
        pub(crate) unk_b0c: u32,
        pub(crate) unk_b10: u32,
        pub(crate) pad_b14: Pad<0x8>,
        pub(crate) unk_b1c: u32,
        pub(crate) unk_b20: u32,
        pub(crate) unk_b24: u32,
        pub(crate) unk_b28: u32,
        pub(crate) unk_b2c: u32,
        pub(crate) unk_b30: u32,
        pub(crate) unk_b34: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_b38_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_b38_4: u32,

        pub(crate) unk_b38: Array<0xc, u32>,
        pub(crate) unk_b68: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_b6c: Array<0xd0, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_c3c: u32,
    }

    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct GpuQueueStatsVtx {
        pub(crate) busy: u32,
        pub(crate) unk_4: u32,
        pub(crate) cur_cmdqueue: u64,
        pub(crate) cur_count: u32,
        pub(crate) unk_14: u32,
    }

    #[versions(AGX)]
    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct GpuStatsVtx {
        pub(crate) unk_4: u32,
        pub(crate) queues: Array<0x4, GpuQueueStatsVtx>,
        pub(crate) unk_68: Array<0x8, u8>,
        pub(crate) unk_70: u32,
        pub(crate) unk_74: u32,
        pub(crate) unk_timestamp: u64,
        pub(crate) unk_80: Array<0x40, u8>,
    }

    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct GpuQueueStatsFrag {
        pub(crate) busy: u32,
        pub(crate) cur_cmdqueue: u64,
        pub(crate) unk_c: u32,
        pub(crate) unk_10: u32,
        pub(crate) unk_14: Array<0x14, u8>,
    }

    #[versions(AGX)]
    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct GpuStatsFrag {
        pub(crate) unk_0: Array<0x18, u8>,
        pub(crate) queues: Array<0x4, GpuQueueStatsFrag>,
        pub(crate) unk_d0: Array<0x38, u8>,
        pub(crate) tvb_overflows_1: u32,
        pub(crate) tvb_overflows_2: u32,
        pub(crate) unk_f8: u32,
        pub(crate) unk_fc: u32,
        pub(crate) cur_stamp_id: i32,
        pub(crate) unk_104: Array<0x14, u8>,
        pub(crate) unk_118: i32,
        pub(crate) unk_11c: u32,
        pub(crate) unk_120: u32,
        pub(crate) unk_124: u32,
        pub(crate) unk_128: u32,
        pub(crate) unk_12c: u32,
        pub(crate) unk_timestamp: u64,
        pub(crate) unk_134: Array<0x8c, u8>,
    }

    #[versions(AGX)]
    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct GpuGlobalStatsVtx {
        pub(crate) total_cmds: u32,
        pub(crate) stats: GpuStatsVtx::ver,
        #[ver(V >= V13_0B4)]
        pub(crate) unk_pad: Array<0x800, u8>,
    }

    #[versions(AGX)]
    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct GpuGlobalStatsFrag {
        pub(crate) total_cmds: u32,
        pub(crate) unk_4: u32,
        pub(crate) stats: GpuStatsFrag::ver,
        #[ver(V >= V13_0B4)]
        pub(crate) unk_pad: Array<0x800, u8>,
    }

    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct RuntimeScratch {
        pub(crate) unk_280: Array<0x6800, u8>,
        pub(crate) unk_6a80: u32,
        pub(crate) gpu_idle: u32,
        pub(crate) unkpad_6a88: Pad<0x14>,
        pub(crate) unk_6a9c: u32,
        pub(crate) unk_ctr0: u32,
        pub(crate) unk_ctr1: u32,
        pub(crate) unk_6aa8: u32,
        pub(crate) unk_6aac: u32,
        pub(crate) unk_ctr2: u32,
        pub(crate) unk_6ab4: u32,
        pub(crate) unk_6ab8: u32,
        pub(crate) unk_6abc: u32,
        pub(crate) unk_6ac0: u32,
        pub(crate) unk_6ac4: u32,
        pub(crate) unk_ctr3: u32,
        pub(crate) unk_6acc: u32,
        pub(crate) unk_6ad0: u32,
        pub(crate) unk_6ad4: u32,
        pub(crate) unk_6ad8: u32,
        pub(crate) unk_6adc: u32,
        pub(crate) unk_6ae0: u32,
        pub(crate) unk_6ae4: u32,
        pub(crate) unk_6ae8: u32,
        pub(crate) unk_6aec: u32,
        pub(crate) unk_6af0: u32,
        pub(crate) unk_ctr4: u32,
        pub(crate) unk_ctr5: u32,
        pub(crate) unk_6afc: u32,
        pub(crate) pad_6b00: Pad<0x38>,
        pub(crate) unk_6b38: u32,
        pub(crate) pad_6b3c: Pad<0x84>,
    }

    pub(crate) type BufferMgrCtl = Array<4, u32>;

    #[versions(AGX)]
    #[repr(C)]
    pub(crate) struct RuntimePointers<'a> {
        pub(crate) pipes: Array<4, PipeChannels>,

        pub(crate) device_control: ChannelRing<channels::ChannelState, channels::DeviceControlMsg>,
        pub(crate) event: ChannelRing<channels::ChannelState, channels::RawEventMsg>,
        pub(crate) fw_log: ChannelRing<channels::FwLogChannelState, channels::RawFwLogMsg>,
        pub(crate) ktrace: ChannelRing<channels::ChannelState, channels::RawKTraceMsg>,
        pub(crate) stats: ChannelRing<channels::ChannelState, channels::RawStatsMsg::ver>,

        pub(crate) __pad0: Pad<0x50>,
        pub(crate) unk_160: u64,
        pub(crate) unk_168: u64,
        pub(crate) stats_vtx: GpuPointer<'a, super::GpuGlobalStatsVtx::ver>,
        pub(crate) stats_frag: GpuPointer<'a, super::GpuGlobalStatsFrag::ver>,
        pub(crate) stats_comp: GpuPointer<'a, &'a [u8]>,
        pub(crate) hwdata_a: GpuPointer<'a, super::HwDataA::ver>,
        pub(crate) unkptr_190: GpuPointer<'a, &'a [u8]>,
        pub(crate) unkptr_198: GpuPointer<'a, &'a [u8]>,
        pub(crate) hwdata_b: GpuPointer<'a, super::HwDataB::ver>,
        pub(crate) hwdata_b_2: GpuPointer<'a, super::HwDataB::ver>,
        pub(crate) fwlog_ring2: GpuPointer<'a, &'a [channels::RawFwLogMsg]>,
        pub(crate) unkptr_1b8: GpuPointer<'a, &'a [u8]>,
        pub(crate) unkptr_1c0: GpuPointer<'a, &'a [u8]>,
        pub(crate) unkptr_1c8: GpuPointer<'a, &'a [u8]>,
        pub(crate) unk_1d0: u32,
        pub(crate) unk_1d4: u32,
        pub(crate) unk_1d8: Array<0x3c, u8>,
        pub(crate) buffer_mgr_ctl: GpuPointer<'a, &'a [BufferMgrCtl]>,
        pub(crate) buffer_mgr_ctl_2: GpuPointer<'a, &'a [BufferMgrCtl]>,
        pub(crate) __pad1: Pad<0x5c>,
        pub(crate) gpu_scratch: RuntimeScratch,
    }
    #[versions(AGX)]
    no_debug!(RuntimePointers::ver<'_>);

    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct PendingStamp {
        pub(crate) info: u32,
        pub(crate) wait_value: u32,
    }

    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct FaultInfo {
        pub(crate) unk_0: u32,
        pub(crate) unk_4: u32,
        pub(crate) queue_uuid: u32,
        pub(crate) unk_c: u32,
        pub(crate) unk_10: u32,
        pub(crate) unk_14: u32,
    }

    #[versions(AGX)]
    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct GlobalsSub {
        pub(crate) unk_54: u16,
        pub(crate) unk_56: u16,
        pub(crate) unk_58: u16,
        pub(crate) unk_5a: u32,
        pub(crate) unk_5e: u32,
        pub(crate) unk_62: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_66_0: Array<0xc, u8>,

        pub(crate) unk_66: u32,
        pub(crate) unk_6a: Array<0x16, u8>,
    }

    #[versions(AGX)]
    #[derive(Debug, Default)]
    #[repr(C)]
    pub(crate) struct Globals {
        pub(crate) ktrace_enable: u32,
        pub(crate) unk_4: Array<0x24, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_28_0: u32,

        pub(crate) unk_28: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_2c_0: u32,

        pub(crate) unk_2c: u32,
        pub(crate) unk_30: u32,
        pub(crate) unk_34: u32,
        pub(crate) unk_38: Array<0x1c, u8>,

        pub(crate) sub: GlobalsSub::ver,

        pub(crate) unk_80: Array<0xf80, u8>,
        pub(crate) unk_1000: Array<0x7000, u8>,
        pub(crate) unk_8000: Array<0x900, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_8900_0: u32,

        pub(crate) unk_8900: u32,
        pub(crate) unk_atomic: u32,
        pub(crate) unk_8908: u32,
        pub(crate) unk_890c: u32,
        pub(crate) unk_8910: u32,
        pub(crate) unk_8914: u32,
        pub(crate) unk_8918: u32,
        pub(crate) unk_891c: u32,
        pub(crate) unk_8920: u32,
        pub(crate) unk_8924: u32,
        pub(crate) unk_8928: u32,
        pub(crate) unk_892c: F32,
        pub(crate) unk_8930: F32,
        pub(crate) unk_8934: u32,
        pub(crate) unk_8938: u32,
        pub(crate) unk_893c: u32,
        pub(crate) unk_8940: u32,
        pub(crate) unk_8944: u32,
        pub(crate) unk_8948: u32,
        pub(crate) unk_894c: u32,
        pub(crate) unk_8950: Array<0x6c, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_89bc_0: Array<0x3c, u8>,

        pub(crate) unk_89bc: u32,
        pub(crate) unk_89c0: u32,
        pub(crate) unk_89c4: i32,
        pub(crate) unk_89c8: u32,
        pub(crate) unk_89cc: F32,
        pub(crate) unk_89d0: F32,
        pub(crate) unk_89d4: Array<0xc, u8>,
        pub(crate) unk_89e0: u32,
        pub(crate) unk_89e4: u32,
        pub(crate) unk_89e8: F32,
        pub(crate) unk_89ec: F32,
        pub(crate) unk_89f0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_89f4_0: Array<0x8, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_89f4_8: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_89f4_c: Array<0x50, u8>,

        pub(crate) hws1: HwDataShared1,
        pub(crate) hws2: HwDataShared2,
        pub(crate) unk_8fac: Array<0x60, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_900c_0: Array<0x28, u8>,

        pub(crate) unk_900c: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_9010_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_9010_4: Array<0x14, u8>,

        pub(crate) unk_9010: Array<0x2c, u8>,
        pub(crate) unk_903c: u32,
        pub(crate) unk_9040: Array<0xc0, u8>,
        pub(crate) unk_9100: Array<0x6f00, u8>,
        pub(crate) unk_10000: Array<0xe50, u8>,
        pub(crate) unk_10e50: u32,
        pub(crate) unk_10e54: Array<0x2c, u8>,
        pub(crate) unk_10e80: u32,
        pub(crate) do_init: u32,
        pub(crate) unk_10e88: Array<0x188, u8>,
        pub(crate) idle_ts: u64,
        pub(crate) idle_unk: u64,
        pub(crate) unk_11020: u32,
        pub(crate) unk_11024: u32,
        pub(crate) unk_11028: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_1102c_0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_1102c_4: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_1102c_8: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_1102c_c: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_1102c_10: u32,

        pub(crate) unk_1102c: u32,
        pub(crate) idle_to_off_timeout_ms: u32,
        pub(crate) unk_11034: u32,
        pub(crate) unk_11038: u32,
        pub(crate) pending_stamps: Array<0x110, PendingStamp>,
        pub(crate) unk_117bc: u32,
        pub(crate) fault_info: FaultInfo,
        pub(crate) counter: u32,
        pub(crate) unk_118dc: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_118e0_0: Array<0x9c, u8>,

        pub(crate) unk_118e0: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_118e4_0: u32,

        pub(crate) unk_118e4: Array<0x1c, u8>,
        pub(crate) unk_11900: Array<0x440, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_11d40: Array<0x19c, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_11edc: u32,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_11ee0: Array<0x1c, u8>,

        #[ver(V >= V13_0B4)]
        pub(crate) unk_11efc: u32,
    }

    #[derive(Debug, Default, Clone, Copy)]
    #[repr(C, packed)]
    pub(crate) struct UatLevelInfo {
        pub(crate) unk_3: u8,
        pub(crate) unk_1: u8,
        pub(crate) unk_2: u8,
        pub(crate) index_shift: u8,
        pub(crate) num_entries: u16,
        pub(crate) unk_4: u16,
        pub(crate) unk_8: u64,
        pub(crate) unk_10: u64,
        pub(crate) index_mask: u64,
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct InitData<'a> {
        #[ver(V >= V13_0B4)]
        pub(crate) ver_info: Array<0x4, u16>,

        pub(crate) unk_buf: GpuPointer<'a, &'a [u8]>,
        pub(crate) unk_8: u32,
        pub(crate) unk_c: u32,
        pub(crate) runtime_pointers: GpuPointer<'a, super::RuntimePointers::ver>,
        pub(crate) globals: GpuPointer<'a, super::Globals::ver>,
        pub(crate) fw_status: GpuPointer<'a, super::FwStatus>,
        pub(crate) uat_page_size: u16,
        pub(crate) uat_page_bits: u8,
        pub(crate) uat_num_levels: u8,
        pub(crate) uat_level_info: Array<0x3, UatLevelInfo>,
        pub(crate) __pad0: Pad<0x14>,
        pub(crate) host_mapped_fw_allocations: u32,
        pub(crate) unk_ac: u32,
        pub(crate) unk_b0: u32,
        pub(crate) unk_b4: u32,
        pub(crate) unk_b8: u32,
    }
}

#[derive(Debug)]
pub(crate) struct ChannelRing<T: GpuStruct + Debug + Default, U: Copy>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug,
{
    pub(crate) state: GpuObject<T>,
    pub(crate) ring: GpuArray<U>,
}

impl<T: GpuStruct + Debug + Default, U: Copy> ChannelRing<T, U>
where
    for<'a> <T as GpuStruct>::Raw<'a>: Debug,
{
    pub(crate) fn to_raw(&self) -> raw::ChannelRing<T, U> {
        raw::ChannelRing {
            state: Some(self.state.weak_pointer()),
            ring: Some(self.ring.weak_pointer()),
        }
    }
}

#[derive(Debug)]
pub(crate) struct FwStatus {
    pub(crate) channel: ChannelRing<channels::FwCtlChannelState, channels::FwCtlMsg>,
}

impl GpuStruct for FwStatus {
    type Raw<'a> = raw::FwStatus;
}

#[versions(AGX)]
#[derive(Debug, Default)]
pub(crate) struct GpuGlobalStatsVtx {}

#[versions(AGX)]
impl GpuStruct for GpuGlobalStatsVtx::ver {
    type Raw<'a> = raw::GpuGlobalStatsVtx::ver;
}

#[versions(AGX)]
#[derive(Debug, Default)]
pub(crate) struct GpuGlobalStatsFrag {}

#[versions(AGX)]
impl GpuStruct for GpuGlobalStatsFrag::ver {
    type Raw<'a> = raw::GpuGlobalStatsFrag::ver;
}

#[versions(AGX)]
#[derive(Debug, Default)]
pub(crate) struct HwDataA {}

#[versions(AGX)]
impl GpuStruct for HwDataA::ver {
    type Raw<'a> = raw::HwDataA::ver;
}

#[versions(AGX)]
#[derive(Debug, Default)]
pub(crate) struct HwDataB {}

#[versions(AGX)]
impl GpuStruct for HwDataB::ver {
    type Raw<'a> = raw::HwDataB::ver;
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct Stats {
    pub(crate) vtx: GpuObject<GpuGlobalStatsVtx::ver>,
    pub(crate) frag: GpuObject<GpuGlobalStatsFrag::ver>,
    pub(crate) comp: GpuArray<u8>,
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct RuntimePointers {
    pub(crate) stats: Stats::ver,

    pub(crate) hwdata_a: GpuObject<HwDataA::ver>,
    pub(crate) unkptr_190: GpuArray<u8>,
    pub(crate) unkptr_198: GpuArray<u8>,
    pub(crate) hwdata_b: GpuObject<HwDataB::ver>,

    pub(crate) fwlog_ring2: GpuArray<channels::RawFwLogMsg>,

    pub(crate) unkptr_1b8: GpuArray<u8>,
    pub(crate) unkptr_1c0: GpuArray<u8>,
    pub(crate) unkptr_1c8: GpuArray<u8>,

    pub(crate) buffer_mgr_ctl: GpuArray<raw::BufferMgrCtl>,
}

#[versions(AGX)]
impl GpuStruct for RuntimePointers::ver {
    type Raw<'a> = raw::RuntimePointers::ver<'a>;
}

#[versions(AGX)]
#[derive(Debug, Default)]
pub(crate) struct Globals {}

#[versions(AGX)]
impl GpuStruct for Globals::ver {
    type Raw<'a> = raw::Globals::ver;
}

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct InitData {
    pub(crate) unk_buf: GpuArray<u8>,
    pub(crate) runtime_pointers: GpuObject<RuntimePointers::ver>,
    pub(crate) globals: GpuObject<Globals::ver>,
    pub(crate) fw_status: GpuObject<FwStatus>,
}

#[versions(AGX)]
impl GpuStruct for InitData::ver {
    type Raw<'a> = raw::InitData::ver<'a>;
}
