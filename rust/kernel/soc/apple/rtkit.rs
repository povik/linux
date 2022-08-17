// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]

//! Support for Apple RTKit coprocessors.
//!
//! C header: [`include/linux/soc/apple/rtkit.h`](../../../../include/linux/gpio/driver.h)

use crate::{
    bindings, device,
    error::{code::*, from_kernel_err_ptr, to_result, Result},
    str::CStr,
    types::PointerWrapper,
};

use core::marker::PhantomData;
use core::ptr;

use macros::vtable;

pub struct ShMem(bindings::apple_rtkit_shmem);

#[vtable]
pub trait Operations {
    type Data: PointerWrapper + Send + Sync;

    fn crashed(_data: <Self::Data as PointerWrapper>::Borrowed<'_>) {}

    fn recv_message(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _endpoint: u8,
        _message: u64,
    ) {
    }

    fn recv_message_early(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _endpoint: u8,
        _message: u64,
    ) -> bool {
        return false;
    }

    fn shmem_setup(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _buffer: &mut ShMem,
    ) -> Result {
        Err(EINVAL)
    }

    fn shmem_destroy(_data: <Self::Data as PointerWrapper>::Borrowed<'_>, _buffer: &mut ShMem) {}
}

/// Represents `struct apple_rtkit *`.
///
/// # Invariants
///
/// The pointer is valid.
pub struct RTKit<T: Operations>(*mut bindings::apple_rtkit, PhantomData<T>);

unsafe extern "C" fn crashed_callback<T: Operations>(cookie: *mut core::ffi::c_void) {
    T::crashed(unsafe { T::Data::borrow(cookie) });
}

unsafe extern "C" fn recv_message_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    endpoint: u8,
    message: u64,
) {
    T::recv_message(unsafe { T::Data::borrow(cookie) }, endpoint, message);
}

unsafe extern "C" fn recv_message_early_callback<T: Operations>(
    cookie: *mut core::ffi::c_void,
    endpoint: u8,
    message: u64,
) -> bool {
    T::recv_message_early(unsafe { T::Data::borrow(cookie) }, endpoint, message)
}

unsafe extern "C" fn shmem_setup_callback<T: Operations>(
    _cookie: *mut core::ffi::c_void,
    _bfr: *mut bindings::apple_rtkit_shmem,
) -> core::ffi::c_int {
    panic!("TODO");
}

unsafe extern "C" fn shmem_destroy_callback<T: Operations>(
    _cookie: *mut core::ffi::c_void,
    _bfr: *mut bindings::apple_rtkit_shmem,
) {
    todo!();
}

impl<T: Operations> RTKit<T> {
    const VTABLE: bindings::apple_rtkit_ops = bindings::apple_rtkit_ops {
        crashed: Some(crashed_callback::<T>),
        recv_message: Some(recv_message_callback::<T>),
        recv_message_early: Some(recv_message_early_callback::<T>),
        shmem_setup: if T::HAS_SHMEM_SETUP {
            Some(shmem_setup_callback::<T>)
        } else {
            None
        },
        shmem_destroy: if T::HAS_SHMEM_DESTROY {
            Some(shmem_destroy_callback::<T>)
        } else {
            None
        },
    };

    pub unsafe fn new(
        dev: &dyn device::RawDevice,
        mbox_name: Option<&'static CStr>,
        mbox_idx: usize,
        data: T::Data,
    ) -> Result<Self> {
        let rtk = unsafe {
            from_kernel_err_ptr(bindings::apple_rtkit_init(
                dev.raw_device(),
                data.into_pointer() as *mut core::ffi::c_void,
                match mbox_name {
                    Some(s) => s.as_char_ptr(),
                    None => ptr::null(),
                },
                mbox_idx.try_into()?,
                &Self::VTABLE,
            ))
        }?;

        Ok(Self(rtk, PhantomData))
    }

    pub fn boot(&mut self) -> Result {
        to_result(unsafe { bindings::apple_rtkit_boot(self.0) })
    }
}

// SAFETY: `RTKit` operations require a mutable reference
unsafe impl<T: Operations> Sync for RTKit<T> {}

// SAFETY: `RTKit` operations require a mutable reference
unsafe impl<T: Operations> Send for RTKit<T> {}

impl<T: Operations> Drop for RTKit<T> {
    fn drop(&mut self) {
        // SAFETY: The pointer is valid by the type invariant.
        unsafe { bindings::apple_rtkit_free(self.0) };
    }
}
