// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]
#![allow(clippy::unusual_byte_groupings)]

//! GPU initialization data builder

use crate::fw::channels;
use crate::fw::initdata::*;
use crate::fw::types::*;
use crate::{box_in_place, const_f32, place};
use crate::{gpu, hw};
use kernel::error::Result;
use kernel::macros::versions;

#[versions(AGX)]
pub(crate) struct InitDataBuilder<'a> {
    alloc: &'a mut gpu::KernelAllocators,
    cfg: &'a hw::HwConfig,
    dyncfg: &'a hw::HwDynConfig,
}

#[versions(AGX)]
impl<'a> InitDataBuilder::ver<'a> {
    pub(crate) fn new(
        alloc: &'a mut gpu::KernelAllocators,
        cfg: &'a hw::HwConfig,
        dyncfg: &'a hw::HwDynConfig,
    ) -> InitDataBuilder::ver<'a> {
        InitDataBuilder::ver { alloc, cfg, dyncfg }
    }

    #[inline(never)]
    fn hw_shared1() -> raw::HwDataShared1 {
        raw::HwDataShared1 {
            unk_0: 0,
            unk_4: 0xffffffff,
            unk_8: 0x7282,
            unk_c: 0x50ea,
            unk_10: 0x370a,
            unk_14: 0x25be,
            unk_18: 0x1c1f,
            unk_1c: 0x16fb,
            unk_20: Array::new([0xff; 0x26]),
            unk_a4: 0xffff,
            unk_a8: 0,
            ..Default::default()
        }
    }

    #[inline(never)]
    fn hw_shared2() -> Result<Box<raw::HwDataShared2>> {
        Ok(box_in_place!(raw::HwDataShared2 {
            unk_ac: 0x800,
            unk_b0: 0x1555,
            unk_b4: Array::new([0xff; 24]),
            unk_d4: Array::new([0xff; 16]),
            unk_5b4: 0xc0007,
            ..Default::default()
        })?)
    }

    #[inline(never)]
    fn hwdata_a(&mut self) -> Result<GpuObject<HwDataA::ver>> {
        self.alloc
            .shared
            .new_inplace(Default::default(), |_inner, ptr| {
                let raw = place!(
                    ptr,
                    raw::HwDataA::ver {
                        unk_4: 192000,
                        #[ver(V >= V13_0B4)]
                        unk_8_0: 192000,
                        pwr_status: AtomicU32::new(4),
                        unk_10: const_f32!(1.0),
                        actual_pstate: 1,
                        tgt_pstate: 1,
                        unk_3c: 300,
                        unk_40: 1,
                        unk_44: 600,
                        unk_4c: 100,
                        // perf related,
                        unk_64c: 625,
                        unk_658: const_f32!(0.99680513),
                        unk_660: const_f32!(0.0031948888),
                        // gpu-pwr-integral-gain
                        unk_668: const_f32!(0.0202129),
                        unk_674: const_f32!(19551.0),
                        // gpu-pwr-proportional-gain
                        unk_678: const_f32!(5.2831855),
                        unk_680: 0xbcfb676e,
                        unk_684: 0xfffffdd0,
                        unk_68c: 600,
                        unk_698: 19551,
                        unk_6b8: 600,
                        unk_6d4: 48,
                        unk_6e0: const_f32!(0.9166667),
                        unk_6e8: const_f32!(0.08333333),
                        // gpu-ppm-ki / gpu-avg-power-target-filter-tc?
                        unk_6f0: const_f32!(0.732),
                        #[ver(V >= V13_0B4)]
                        unk_6fc: const_f32!(65536.0),
                        #[ver(V < V13_0B4)]
                        unk_6fc: const_f32!(0.0),
                        // gpu-ppm-kp
                        unk_700: const_f32!(6.9),
                        // gpu-pwr-min-duty-cycle?
                        unk_70c: 40,
                        unk_710: 600,
                        unk_71c: const_f32!(0.0),
                        unk_720: 19551,
                        cur_power_mw_2: 0x0,
                        unk_728: 100,
                        #[ver(V >= V13_0B4)]
                        unk_730_0: 0x232800,
                        // gpu-perf-tgt-utilization
                        unk_75c: 85,
                        unk_764: 100,
                        unk_768: 25,
                        unk_76c: 6,
                        pad_770: 0x0,
                        unk_774: 6,
                        unk_778: 1,
                        unk_780: const_f32!(0.8),
                        unk_784: const_f32!(0.98),
                        unk_788: const_f32!(0.2),
                        unk_78c: const_f32!(0.02),
                        unk_790: const_f32!(7.8956833),
                        // gpu-perf-integral-gain2
                        unk_794: const_f32!(0.197392),
                        unk_79c: const_f32!(95.0),
                        unk_7a0: const_f32!(14.707963),
                        // gpu-perf-proportional-gain2
                        unk_7a4: const_f32!(6.853981),
                        unk_7a8: const_f32!(3.1578948),
                        unk_7ac: 300,
                        unk_7b0: 600,
                        unk_7b4: 300,
                        unk_7c0: 0x55,
                        unk_7e0: 300,
                        unk_7fc: const_f32!(65536.0),
                        unk_800: const_f32!(40.0),
                        unk_804: const_f32!(600.0),
                        unk_808: 0x4fe,
                        // gpu-pwr-min-duty-cycle?
                        unk_818: 40,
                        unk_824: const_f32!(100.0),
                        unk_828: 600,
                        unk_830: const_f32!(0.8),
                        unk_834: const_f32!(0.2),
                        unk_870: 0x12,
                        unk_878: 0x1f40,
                        unk_87c: 0xffffff24,
                        unk_880: 0x4,
                        unk_894: const_f32!(1.0),

                        unk_89c: const_f32!(1.6),
                        unk_8a8: const_f32!(65536.0),
                        // gpu-fast-die0-proportional-gain?
                        unk_8ac: const_f32!(5.0),
                        // gpu-pwr-min-duty-cycle?,
                        unk_8b8: 40,
                        unk_8bc: 600,
                        unk_8c0: 600,
                        unk_8cc: 9880,
                        unk_8ec: 600,
                        unk_b94: 600,
                        unk_c2c: 1,
                        unk_c30: 1,
                        unk_c34: 19551,
                        unk_c38: 19551,
                        unk_c3c: 19551,
                        unk_c48: const_f32!(0.992),
                        unk_c4c: const_f32!(0.008),
                        unk_c50: 500,
                        unk_c54: 1000,
                        #[ver(V >= V13_0B4)]
                        unk_c58_0: 24000000,
                        unk_c5c: 30000,
                        unk_c60: 29900,
                        unk_c64: 27500,
                        unk_c68: 55000,
                        #[ver(V >= V13_0B4)]
                        unk_c6c_0: 1320000000,
                        unk_c6c: const_f32!(0.99985456),
                        unk_c70: const_f32!(0.00014545454),
                        unk_cf8: 500,
                        unk_d04: const_f32!(0.992),
                        unk_d0c: const_f32!(0.008),
                        unk_d14: const_f32!(0.06),
                        unk_d20: const_f32!(65536.0),
                        unk_d24: const_f32!(4.0),
                        unk_d30: 0x28,
                        unk_d34: 600,
                        unk_d38: 600,
                        unk_d40: const_f32!(19551.0),
                        unk_d44: 19551,
                        unk_d4c: 1000,
                        #[ver(V >= V13_0B4)]
                        unk_d54_0: 24000000,
                        unk_d64: 600,
                        unk_d8c: 0x80000000,
                        unk_d90: 4,
                        unk_d9c: const_f32!(0.6),
                        unk_da4: const_f32!(0.4),
                        unk_dac: const_f32!(0.38552),
                        unk_db8: const_f32!(65536.0),
                        unk_dbc: const_f32!(13.56),
                        unk_dcc: 600,
                        #[ver(V >= V13_0B4)]
                        unk_e10_0: raw::HwDataA130Extra {
                            unk_38: 4,
                            unk_3c: 8000,
                            unk_40: 2500,
                            unk_48: 0xffffffff,
                            unk_4c: 50,
                            unk_54: 50,
                            unk_58: 0x1,
                            unk_60: const_f32!(0.88888888),
                            unk_64: const_f32!(0.66666666),
                            unk_68: const_f32!(0.111111111),
                            unk_6c: const_f32!(0.33333333),
                            unk_70: const_f32!(-0.4),
                            unk_74: const_f32!(-0.8),
                            unk_7c: const_f32!(65536.0),
                            unk_80: const_f32!(-5.0),
                            unk_84: const_f32!(-10.0),
                            unk_8c: 40,
                            unk_90: 600,
                            unk_9c: const_f32!(8000.0),
                            unk_a0: 1400,
                            unk_a8: 72,
                            unk_ac: 24,
                            unk_b0: 1728000,
                            unk_b8: 576000,
                            unk_c4: const_f32!(65536.0),
                            unk_114: const_f32!(65536.0),
                            unk_124: 40,
                            unk_128: 600,
                            ..Default::default()
                        },
                        unk_e10: Array::new([
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x12, 0x0,
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x70, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0,
                            0x0, 0x0, 0x0, 0x0,
                        ]),
                        unk_1610: Array::new([
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                            0x12, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0,
                        ]),
                        #[ver(V < V13_0B4)]
                        unk_1638: Array::new([0, 0, 0, 0, 1, 0, 0, 0]),
                        hws1: Self::hw_shared1(),
                        hws2: *Self::hw_shared2()?,
                        unk_3ce0: Array::new([
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7a, 0x44, 0x0, 0x0, 0x0, 0x0,
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x34, 0x42,
                            0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                        ]),
                        ..Default::default()
                    }
                );

                for i in 0..self.cfg.perf_states.len() {
                    raw.unk_74[i] = self.cfg.k;
                }

                Ok(raw)
            })
    }

    #[inline(never)]
    fn hwdata_b(&mut self) -> Result<GpuObject<HwDataB::ver>> {
        self.alloc
            .shared
            .new_inplace(Default::default(), |_inner, ptr| {
                let raw = place!(
                    ptr,
                    raw::HwDataB::ver {
                        // Userspace VA map related
                        #[ver(V < V13_0B4)]
                        unk_0: 0x13_00000000,
                        unk_8: 0x14_00000000,
                        #[ver(V < V13_0B4)]
                        unk_10: 0x1_00000000,
                        unk_18: 0xffc00000,
                        unk_20: 0x11_00000000,
                        unk_28: 0x11_00000000,
                        // userspace address?
                        unk_30: 0x6f_ffff8000,
                        // unmapped?
                        unkptr_38: 0xffffffa0_11800000,
                        // TODO: yuv matrices
                        unk_454: 0x1,
                        unk_458: 0x1,
                        unk_460: 0x1,
                        unk_464: 0x1,
                        unk_468: 0x1,
                        unk_47c: 0x1,
                        unk_484: 0x1,
                        unk_48c: 0x1,
                        unk_490: 24000,
                        unk_494: 0x8,
                        unk_49c: 0x1,
                        unk_4a0: 0x1,
                        unk_4a4: 0x1,
                        unk_4c0: 0x1f,
                        unk_4f0: 0x1,
                        unk_4f4: 0x1,
                        unk_504: 0x31,
                        unk_524: 0x1,
                        unk_53c: 0x8,
                        unk_554: 0x1,
                        uat_context_table_base: self.dyncfg.uat_context_table_base,
                        unk_560: 0xb,
                        unk_564: 0x4,
                        unk_568: 0x8,
                        max_pstate: 0x4,
                        #[ver(V < V13_0B4)]
                        num_pstates: 0x7,
                        #[ver(V >= V13_0B4)]
                        unk_a84: 0x24,
                        #[ver(V < V13_0B4)]
                        unk_a84: 27,
                        unk_a88: 73,
                        unk_a8c: 100,

                        #[ver(V < V13_0B4)]
                        min_volt: 850,
                        #[ver(V < V13_0B4)]
                        unk_ab8: 72,
                        #[ver(V < V13_0B4)]
                        unk_abc: 8,
                        #[ver(V < V13_0B4)]
                        unk_ac0: 0x1020,

                        #[ver(V >= V13_0B4)]
                        unk_ae4: Array::new([0x0, 0x3, 0x7, 0x7]),
                        #[ver(V < V13_0B4)]
                        unk_ae4: Array::new([0x0, 0xf, 0x3f, 0x3f]),
                        unk_b10: 0x1,
                        unk_b24: 0x1,
                        unk_b28: 0x1,
                        unk_b2c: 0x1,
                        #[ver(V >= V13_0B4)]
                        unk_b38_0: 1,
                        #[ver(V >= V13_0B4)]
                        unk_b38_4: 1,
                        unk_b38: Array::new([0xffffffff; 12]),
                        #[ver(V >= V13_0B4)]
                        unk_c3c: 0x19,
                        ..Default::default()
                    }
                );

                raw.chip_id = self.cfg.chip_id;

                raw.max_pstate = self.cfg.perf_states.len() as u32 - 1;
                #[ver(V < V13_0B4)]
                {
                    raw.num_pstates = self.cfg.perf_states.len() as u32;
                    raw.min_volt = self.cfg.min_volt;
                }
                for (i, ps) in self.cfg.perf_states.iter().enumerate() {
                    raw.frequencies[i] = ps.1 / 1000000;
                    raw.voltages[i] = [ps.2; 8];
                    let vm = self.cfg.min_volt.max(ps.2);
                    raw.voltages_sram[i] = [vm, 0, 0, 0, 0, 0, 0, 0];
                    raw.unk_9b4[i] = self.cfg.k;
                    raw.perf_levels[i] = ps.0;
                }

                Ok(raw)
            })
    }

    #[inline(never)]
    fn globals(&mut self) -> Result<GpuObject<Globals::ver>> {
        self.alloc
            .shared
            .new_inplace(Default::default(), |_inner, ptr| {
                Ok(place!(
                    ptr,
                    raw::Globals::ver {
                        //ktrace_enable: 0xffffffff,
                        ktrace_enable: 0,
                        #[ver(V >= V13_0B4)]
                        unk_28_0: 0, // debug
                        unk_28: 1,
                        #[ver(V >= V13_0B4)]
                        unk_2c_0: 0,
                        unk_2c: 1,
                        #[ver(V >= V13_0B4)]
                        unk_30: 0,
                        #[ver(V < V13_0B4)]
                        unk_30: 1,
                        unk_34: 120,
                        sub: raw::GlobalsSub::ver {
                            unk_54: 0xffff,
                            unk_56: 40,
                            unk_58: 0xffff,
                            unk_5e: 1,
                            unk_66: 1,
                            ..Default::default()
                        },
                        unk_8900: 1,
                        unk_8908: 19551,
                        unk_890c: 600,
                        unk_8910: 600,
                        unk_891c: 600,
                        unk_8924: 1,
                        // gpu-avg-power-target-filter-tc?
                        unk_8928: 125,
                        // gpu-avg-power-ki-only / gpu-avg-power-target-filter-tc?
                        unk_892c: const_f32!(0.06),
                        // gpu-avg-power-kp
                        unk_8930: const_f32!(4.0),
                        // gpu-avg-power-min-duty-cycle
                        unk_8934: 40,
                        // gpu-avg-power-target-filter-tc
                        unk_8938: 125,
                        #[ver(V >= V13_0B4)]
                        unk_893c: 30000,
                        #[ver(V < V13_0B4)]
                        unk_893c: 29520,
                        // gpu-power-zone-target-0 - gpu-power-zone-target-offset-0
                        unk_8940: 29900,
                        // gpu-power-zone-filter-tc-0
                        unk_8944: 6875,
                        unk_89bc: 9880,
                        unk_89c0: 8000,
                        unk_89c4: -220,
                        // gpu-fast-die0-proportional-gain?
                        unk_89cc: const_f32!(5.0),
                        unk_89d0: const_f32!(1.6),
                        unk_89e0: 1,
                        unk_89e4: 19551,
                        // gpu-ppm-kp
                        unk_89e8: const_f32!(6.9),
                        // gpu-ppm-ki / gpu-avg-power-target-filter-tc?
                        unk_89ec: const_f32!(0.732),
                        #[ver(V >= V13_0B4)]
                        unk_89f4_8: 1,
                        hws1: Self::hw_shared1(),
                        hws2: *Self::hw_shared2()?,
                        unk_900c: 1,
                        #[ver(V >= V13_0B4)]
                        unk_9010_0: 1,
                        #[ver(V >= V13_0B4)]
                        unk_903c: 1,
                        #[ver(V < V13_0B4)]
                        unk_903c: 0,
                        unk_10e80: 11,
                        do_init: 1,
                        unk_11020: 40,
                        unk_11024: 10,
                        unk_11028: 250,
                        #[ver(V >= V13_0B4)]
                        unk_1102c_0: 1,
                        #[ver(V >= V13_0B4)]
                        unk_1102c_4: 1,
                        #[ver(V >= V13_0B4)]
                        unk_1102c_8: 100,
                        #[ver(V >= V13_0B4)]
                        unk_1102c_c: 1,
                        idle_to_off_timeout_ms: 2,
                        unk_11034: 40,
                        unk_11038: 5,
                        unk_118e0: 40,
                        #[ver(V >= V13_0B4)]
                        unk_118e4_0: 50,
                        #[ver(V >= V13_0B4)]
                        unk_11edc: 8,
                        #[ver(V >= V13_0B4)]
                        unk_11efc: 8,
                        ..Default::default()
                    }
                ))
            })
    }

    #[inline(never)]
    fn make_channel<T: GpuStruct + Debug + Default, U: Copy + Default>(
        &mut self,
        count: usize,
        rx: bool,
    ) -> Result<ChannelRing<T, U>>
    where
        for<'b> <T as GpuStruct>::Raw<'b>: Default + Debug,
    {
        let ring_alloc = if rx {
            &mut self.alloc.shared
        } else {
            &mut self.alloc.private
        };

        let ring = ring_alloc.array_empty(count)?;

        Ok(ChannelRing {
            state: self
                .alloc
                .shared
                .new_object(Default::default(), |_inner| Default::default())?,
            ring,
        })
    }

    #[inline(never)]
    fn runtime_pointers(&mut self) -> Result<GpuObject<RuntimePointers::ver>> {
        let hwa = self.hwdata_a()?;
        let hwb = self.hwdata_b()?;

        let pointers: Box<RuntimePointers::ver> = box_in_place!(RuntimePointers::ver {
            stats: Stats::ver {
                vtx: self.alloc.shared.new_default::<GpuGlobalStatsVtx::ver>()?,
                frag: self.alloc.shared.new_inplace(
                    Default::default(),
                    |_inner, ptr: *mut MaybeUninit<raw::GpuGlobalStatsFrag::ver>| {
                        Ok(place!(
                            ptr,
                            raw::GpuGlobalStatsFrag::ver {
                                stats: raw::GpuStatsFrag::ver {
                                    cur_stamp_id: -1,
                                    unk_118: -1,
                                    ..Default::default()
                                },
                                ..Default::default()
                            }
                        ))
                    },
                )?,
                comp: self.alloc.shared.array_empty(0x980)?,
            },

            hwdata_a: hwa,
            unkptr_190: self.alloc.shared.array_empty(0x80)?,
            unkptr_198: self.alloc.shared.array_empty(0xc0)?,
            hwdata_b: hwb,
            fwlog_ring2: self.alloc.shared.array_empty(0x600)?,

            unkptr_1b8: self.alloc.shared.array_empty(0x1000)?,
            unkptr_1c0: self.alloc.shared.array_empty(0x300)?,
            unkptr_1c8: self.alloc.shared.array_empty(0x1000)?,

            buffer_mgr_ctl: self.alloc.gpu.array_empty(126)?,
        })?;

        self.alloc.shared.new_boxed(pointers, |inner, ptr| {
            Ok(place!(
                ptr,
                raw::RuntimePointers::ver {
                    pipes: Default::default(),
                    device_control: Default::default(),
                    event: Default::default(),
                    fw_log: Default::default(),
                    ktrace: Default::default(),
                    stats: Default::default(),

                    stats_vtx: inner.stats.vtx.gpu_pointer(),
                    stats_frag: inner.stats.frag.gpu_pointer(),
                    stats_comp: inner.stats.comp.gpu_pointer(),

                    hwdata_a: inner.hwdata_a.gpu_pointer(),
                    unkptr_190: inner.unkptr_190.gpu_pointer(),
                    unkptr_198: inner.unkptr_198.gpu_pointer(),
                    hwdata_b: inner.hwdata_b.gpu_pointer(),
                    hwdata_b_2: inner.hwdata_b.gpu_pointer(),

                    fwlog_ring2: inner.fwlog_ring2.gpu_pointer(),

                    unkptr_1b8: inner.unkptr_1b8.gpu_pointer(),
                    unkptr_1c0: inner.unkptr_1c0.gpu_pointer(),
                    unkptr_1c8: inner.unkptr_1c8.gpu_pointer(),

                    buffer_mgr_ctl: inner.buffer_mgr_ctl.gpu_pointer(),
                    buffer_mgr_ctl_2: inner.buffer_mgr_ctl.gpu_pointer(),

                    __pad0: Default::default(),
                    unk_160: 0,
                    unk_168: 0,
                    unk_1d0: 0,
                    unk_1d4: 0,
                    unk_1d8: Default::default(),

                    __pad1: Default::default(),
                    gpu_scratch: raw::RuntimeScratch {
                        unk_6b38: 0xff,
                        ..Default::default()
                    },
                }
            ))
        })
    }

    #[inline(never)]
    fn fw_status(&mut self) -> Result<GpuObject<FwStatus>> {
        let channel =
            self.make_channel::<channels::FwCtlChannelState, channels::FwCtlMsg>(0x100, false)?;

        self.alloc
            .shared
            .new_object(FwStatus { channel }, |inner| raw::FwStatus {
                fwctl_channel: inner.channel.to_raw(),
                flags: Default::default(),
            })
    }

    #[inline(never)]
    fn uat_level_info(index_shift: usize, num_entries: usize) -> raw::UatLevelInfo {
        raw::UatLevelInfo {
            index_shift: index_shift as _,
            unk_1: 14,
            unk_2: 14,
            unk_3: 8,
            unk_4: 0x4000,
            num_entries: num_entries as _,
            unk_8: 1,
            unk_10: 0xffffffc000,
            index_mask: ((num_entries - 1) << index_shift) as u64,
        }
    }

    #[inline(never)]
    pub(crate) fn build(&mut self) -> Result<GpuObject<InitData::ver>> {
        let inner: Box<InitData::ver> = box_in_place!(InitData::ver {
            unk_buf: self.alloc.shared.array_empty(0x4000)?,
            runtime_pointers: self.runtime_pointers()?,
            globals: self.globals()?,
            fw_status: self.fw_status()?,
        })?;

        self.alloc.shared.new_boxed(inner, |inner, ptr| {
            Ok(place!(
                ptr,
                raw::InitData::ver {
                    #[ver(V >= V13_0B4)]
                    ver_info: Array::new([1, 1, 16, 1]),
                    unk_buf: inner.unk_buf.gpu_pointer(),
                    unk_8: 0,
                    unk_c: 0,
                    runtime_pointers: inner.runtime_pointers.gpu_pointer(),
                    globals: inner.globals.gpu_pointer(),
                    fw_status: inner.fw_status.gpu_pointer(),
                    uat_page_size: 0x4000,
                    uat_page_bits: 14,
                    uat_num_levels: 3,
                    uat_level_info: Array::new([
                        Self::uat_level_info(36, 8),
                        Self::uat_level_info(25, 2048),
                        Self::uat_level_info(14, 2048),
                    ]),
                    __pad0: Default::default(),
                    host_mapped_fw_allocations: 1,
                    unk_ac: 0,
                    unk_b0: 0,
                    unk_b4: 0,
                    unk_b8: 0,
                }
            ))
        })
    }
}
