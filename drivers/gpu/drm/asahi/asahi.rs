// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

mod alloc;
mod driver;
mod fw;
mod gem;
mod initdata;
mod mmu;
mod object;
