// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![recursion_limit = "1024"]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

mod alloc;
mod channel;
mod driver;
mod fw;
mod gem;
mod gpu;
mod hw;
mod initdata;
mod mmu;
mod object;
mod place;
mod workqueue;

use kernel::module_platform_driver;

module_platform_driver! {
    type: driver::AsahiDevice,
    name: "asahi",
    license: "Dual MIT/GPL",
}
