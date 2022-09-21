// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(dead_code)]

//! GPU Micro operation sequence builder

use crate::fw::microseq;
use crate::fw::types::*;
use kernel::prelude::*;

type MicroSequence = GpuArray<u8>;

struct Builder {
    ops: Vec<u8>,
}

impl Builder {
    fn new() -> Builder {
        Builder { ops: Vec::new() }
    }

    fn add<T: microseq::Operation>(&mut self, op: T) -> Result {
        let p: *const T = &op;
        let p: *const u8 = p as *const u8;
        let s: &[u8] = unsafe { core::slice::from_raw_parts(p, core::mem::size_of::<T>()) };
        self.ops.try_extend_from_slice(s)?;
        Ok(())
    }

    fn build(self, alloc: &mut Allocator) -> Result<MicroSequence> {
        let mut array = alloc.array_empty::<u8>(self.ops.len())?;

        array.as_mut_slice().clone_from_slice(self.ops.as_slice());
        Ok(array)
    }
}
