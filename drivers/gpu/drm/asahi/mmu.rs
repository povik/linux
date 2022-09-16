// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(missing_docs)]
#![allow(unused_imports)]
#![allow(dead_code)]

//! Apple AGX UAT (MMU) support

use core::arch::asm;
use core::mem::{size_of, ManuallyDrop};
use core::ops::Deref;
use core::ptr::NonNull;
use core::sync::atomic::{fence, AtomicU32, AtomicU64, AtomicU8, Ordering};

use kernel::{
    bindings, c_str, device,
    drm::{gem, gem::shmem, mm},
    error::{to_result, Result},
    io_pgtable,
    io_pgtable::{prot, AppleUAT, AppleUATCfg, IOPagetable},
    prelude::*,
    str::CString,
    sync::smutex::Mutex,
    sync::Arc,
    PointerWrapper,
};

const PPL_MAGIC: u64 = 0x4b1d000000000002;

const UAT_NUM_CTX: usize = 64;

pub(crate) const UAT_PGBIT: usize = 14;
pub(crate) const UAT_PGSZ: usize = 1 << UAT_PGBIT;
pub(crate) const UAT_PGMSK: usize = UAT_PGSZ - 1;

type Pte = AtomicU64;
const UAT_NPTE: usize = UAT_PGSZ / size_of::<Pte>();

pub(crate) const UAT_IAS: usize = 39;
pub(crate) const UAT_IAS_KERN: usize = 36;
pub(crate) const UAT_OAS: usize = 36;

const IOVA_USER_BASE: usize = UAT_PGSZ;
const IOVA_USER_TOP: usize = (1 << UAT_IAS) - 1;
const IOVA_TTBR1_BASE: usize = 0xffffff8000000000;
const IOVA_KERN_BASE: usize = 0xffffffa000000000;
const IOVA_KERN_TOP: usize = 0xffffffafffffffff;

const TTBR_VALID: u64 = 0x1; // BIT(0)
const TTBR_ASID_SHIFT: usize = 48;

const PTE_TABLE: u64 = 0x3; // BIT(0) | BIT(1)

type PhysAddr = bindings::phys_addr_t;

struct UatRegion {
    base: PhysAddr,
    size: usize,
    map: NonNull<core::ffi::c_void>,
}

#[repr(C)]
struct FlushInfo {
    state: AtomicU64,
    addr: AtomicU64,
    size: AtomicU64,
}

#[repr(C)]
struct Handoff {
    magic_ap: AtomicU64,
    magic_fw: AtomicU64,

    lock_ap: AtomicU8,
    lock_fw: AtomicU8,
    // Implicit padding: 2 bytes
    turn: AtomicU32,
    unk: AtomicU32,
    // Implicit padding: 4 bytes
    flush: [FlushInfo; UAT_NUM_CTX + 1],

    unk2: AtomicU8,
    // Implicit padding: 7 bytes
    unk3: AtomicU64,
}

const HANDOFF_SIZE: usize = size_of::<Handoff>();

#[repr(C)]
struct ContextTTBS {
    ttb0: AtomicU64,
    ttb1: AtomicU64,
}

const CONTEXTS_SIZE: usize = UAT_NUM_CTX * size_of::<ContextTTBS>();

// We need at least page 0 (ttb0)
const PAGETABLES_SIZE: usize = UAT_PGSZ;

struct ContextInner {
    dev: device::Device,
    is_kernel: bool,
    min_va: usize,
    max_va: usize,
    page_table: IOPagetable<Uat, AppleUATCfg>,
    mm: mm::Allocator<MappingInner>,
}

impl ContextInner {
    fn map_iova(&self, iova: usize, size: usize) -> Result<usize> {
        if iova < self.min_va || (iova + size - 1) > self.max_va {
            Err(EINVAL)
        } else if self.is_kernel {
            Ok(iova - self.min_va)
        } else {
            Ok(iova)
        }
    }

    fn map_pages(
        &mut self,
        iova: usize,
        paddr: usize,
        pgsize: usize,
        pgcount: usize,
        prot: u32,
    ) -> Result<usize> {
        self.page_table.map_pages(
            self.map_iova(iova, pgsize * pgcount)?,
            paddr,
            pgsize,
            pgcount,
            prot,
        )
    }

    fn unmap_pages(&mut self, iova: usize, pgsize: usize, pgcount: usize) -> Result<usize> {
        Ok(self
            .page_table
            .unmap_pages(self.map_iova(iova, pgsize * pgcount)?, pgsize, pgcount))
    }
}

#[derive(Clone)]
pub(crate) struct Context {
    inner: Arc<Mutex<ContextInner>>,
}

pub(crate) struct MappingInner {
    owner: Arc<Mutex<ContextInner>>,
}

pub(crate) struct Mapping(mm::Node<MappingInner>);

impl Mapping {
    pub(crate) fn iova(&self) -> usize {
        self.0.start() as usize
    }
    pub(crate) fn size(&self) -> usize {
        self.0.size() as usize
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        let mut owner = self.0.owner.lock();
        dev_info!(
            owner.dev,
            "MMU: unmap {:#x}:{:#x}",
            self.iova(),
            self.size()
        );
        // Do not try to unmap guard page (-1)
        if owner
            .unmap_pages(self.iova(), UAT_PGSZ, (self.size() >> UAT_PGBIT) - 1)
            .is_err()
        {
            dev_err!(
                owner.dev,
                "MMU: unmap {:#x}:{:#x} failed",
                self.iova(),
                self.size()
            );
        }
    }
}

pub(crate) struct Uat {
    handoff_rgn: UatRegion,
    pagetables_rgn: UatRegion,
    contexts_rgn: UatRegion,

    kernel_context: Context,
}

impl Drop for UatRegion {
    fn drop(&mut self) {
        // SAFETY: the pointer is valid by the type invariant
        unsafe { bindings::memunmap(self.map.as_ptr()) };
    }
}

impl Handoff {
    fn lock(&self) {
        self.lock_ap.store(1, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        while self.lock_fw.load(Ordering::Relaxed) != 0 {
            if self.turn.load(Ordering::Relaxed) != 0 {
                self.lock_ap.store(0, Ordering::Relaxed);
                while self.turn.load(Ordering::Relaxed) != 0 {}
                self.lock_ap.store(1, Ordering::Relaxed);
                fence(Ordering::SeqCst);
            }
        }
        fence(Ordering::Acquire);
    }

    fn unlock(&self) {
        self.turn.store(1, Ordering::Relaxed);
        self.lock_ap.store(0, Ordering::Release);
    }

    fn init(&self) {
        self.magic_ap.store(PPL_MAGIC, Ordering::Relaxed);
        self.unk.store(0xffffffff, Ordering::Relaxed);
        self.unk3.store(0, Ordering::Relaxed);
        fence(Ordering::SeqCst);

        self.lock();

        while self.magic_fw.load(Ordering::Relaxed) != PPL_MAGIC {}

        self.unlock();

        for i in 0..=UAT_NUM_CTX {
            self.flush[i].state.store(0, Ordering::Relaxed);
            self.flush[i].addr.store(0, Ordering::Relaxed);
            self.flush[i].size.store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
    }
}

impl io_pgtable::FlushOps for Uat {
    type Data = ();

    fn tlb_flush_all(_data: <Self::Data as PointerWrapper>::Borrowed<'_>) {
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
    fn tlb_flush_walk(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _size: usize,
        _granule: usize,
    ) {
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
    fn tlb_add_page(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _iova: usize,
        _granule: usize,
    ) {
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
}

impl Context {
    fn new(dev: device::Device, is_kernel: bool) -> Result<Context> {
        let page_table = AppleUAT::new(
            &dev,
            io_pgtable::Config {
                pgsize_bitmap: UAT_PGSZ,
                ias: if is_kernel { UAT_IAS_KERN } else { UAT_IAS },
                oas: UAT_OAS,
                coherent_walk: true,
                quirks: 0,
            },
            (),
        )?;
        let min_va = if is_kernel {
            IOVA_KERN_BASE
        } else {
            IOVA_USER_BASE
        };
        let max_va = if is_kernel {
            IOVA_KERN_TOP
        } else {
            IOVA_USER_TOP
        };

        let mm = mm::Allocator::new(min_va as u64, (max_va - min_va + 1) as u64)?;

        Ok(Context {
            inner: Arc::try_new(Mutex::new(ContextInner {
                dev,
                min_va,
                max_va,
                is_kernel,
                page_table,
                mm,
            }))?,
        })
    }

    fn ttb(&self) -> u64 {
        self.inner.lock().page_table.cfg().ttbr
    }

    pub(crate) fn map(&self, size: usize, sgt: &mut shmem::SGTableIter<'_>) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let node = inner.mm.insert_node(
            MappingInner {
                owner: self.inner.clone(),
            },
            (size + UAT_PGSZ) as u64, // Add guard page
        )?;

        let mut iova = node.start() as usize;

        for range in sgt {
            let addr = range.dma_address();
            let len = range.dma_len();

            if (addr | len | iova) & UAT_PGMSK != 0 {
                dev_err!(
                    inner.dev,
                    "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned",
                    addr,
                    len,
                    iova
                );
                return Err(EINVAL);
            }

            dev_info!(inner.dev, "MMU: map: {:#x}:{:#x} -> {:#x}", addr, len, iova);

            inner.map_pages(
                iova,
                addr,
                UAT_PGSZ,
                len >> UAT_PGBIT,
                prot::PRIV | prot::READ | prot::WRITE | prot::CACHE,
            )?;

            iova += len;
        }

        Ok(Mapping(node))
    }

    pub(crate) fn map_io(&self, phys: usize, size: usize) -> Result<Mapping> {
        let mut inner = self.inner.lock();

        let node = inner.mm.insert_node(
            MappingInner {
                owner: self.inner.clone(),
            },
            (size + UAT_PGSZ) as u64, // Add guard page
        )?;

        let iova = node.start() as usize;

        if (phys | size | iova) & UAT_PGMSK != 0 {
            dev_err!(
                inner.dev,
                "MMU: Mapping {:#x}:{:#x} -> {:#x} is not page-aligned",
                phys,
                size,
                iova
            );
            return Err(EINVAL);
        }

        dev_info!(
            inner.dev,
            "MMU: IO map: {:#x}:{:#x} -> {:#x}",
            phys,
            size,
            iova
        );

        inner.map_pages(
            iova,
            phys,
            UAT_PGSZ,
            size >> UAT_PGBIT,
            prot::PRIV | prot::READ | prot::WRITE | prot::CACHE | prot::MMIO,
        )?;

        Ok(Mapping(node))
    }
}

impl Uat {
    fn map_region(dev: &dyn device::RawDevice, name: &CStr, size: usize) -> Result<UatRegion> {
        let rdev = dev.raw_device();

        let mut res = core::mem::MaybeUninit::<bindings::resource>::uninit();

        let res = unsafe {
            let idx = bindings::of_property_match_string(
                (*rdev).of_node,
                c_str!("memory-region-names").as_char_ptr(),
                name.as_char_ptr(),
            );
            to_result(idx)?;

            let np = bindings::of_parse_phandle(
                (*rdev).of_node,
                c_str!("memory-region").as_char_ptr(),
                idx,
            );
            if np.is_null() {
                dev_err!(dev, "Missing {} region\n", name);
                return Err(EINVAL);
            }
            let ret = bindings::of_address_to_resource(np, 0, res.as_mut_ptr());
            bindings::of_node_put(np);

            if ret < 0 {
                dev_err!(dev, "Failed to get {} region\n", name);
                to_result(ret)?
            }

            res.assume_init()
        };

        let rgn_size: usize = unsafe { bindings::resource_size(&res) } as usize;

        if size > rgn_size {
            dev_err!(
                dev,
                "Region {} is too small (expected {}, got {})\n",
                name,
                size,
                rgn_size
            );
            return Err(ENOMEM);
        }

        let map = unsafe { bindings::memremap(res.start, rgn_size, bindings::MEMREMAP_WB.into()) };
        let map = NonNull::new(map);

        match map {
            None => {
                dev_err!(dev, "Failed to remap {} region\n", name);
                Err(ENOMEM)
            }
            Some(map) => Ok(UatRegion {
                base: res.start,
                size: rgn_size as usize,
                map,
            }),
        }
    }

    fn handoff(&self) -> &Handoff {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.handoff_rgn.map.as_ptr() as *mut Handoff).as_ref() }.unwrap()
    }

    fn ttbs(&self) -> &[ContextTTBS; UAT_NUM_CTX] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.contexts_rgn.map.as_ptr() as *mut [ContextTTBS; UAT_NUM_CTX]).as_ref() }
            .unwrap()
    }

    fn kpt0(&self) -> &[Pte; UAT_NPTE] {
        // SAFETY: pointer is non-null per the type invariant
        unsafe { (self.pagetables_rgn.map.as_ptr() as *mut [Pte; UAT_NPTE]).as_ref() }.unwrap()
    }

    pub(crate) fn kernel_context(&self) -> &Context {
        &self.kernel_context
    }

    pub(crate) fn new(dev: &dyn device::RawDevice) -> Result<Self> {
        dev_info!(dev, "MMU: Initializing...\n");

        let handoff_rgn = Self::map_region(dev, c_str!("handoff"), HANDOFF_SIZE)?;
        let contexts_rgn = Self::map_region(dev, c_str!("contexts"), CONTEXTS_SIZE)?;
        let pagetables_rgn = Self::map_region(dev, c_str!("pagetables"), PAGETABLES_SIZE)?;

        dev_info!(dev, "MMU: Initializing kernel page table\n");

        let kernel_context = Context::new(device::Device::from_dev(dev), true)?;

        let ttb1 = kernel_context.ttb();

        let uat = Self {
            handoff_rgn,
            pagetables_rgn,
            contexts_rgn,
            kernel_context,
        };

        dev_info!(dev, "MMU: Initializing handoff\n");
        uat.handoff().init();

        dev_info!(dev, "MMU: Initializing TTBs\n");

        uat.handoff().lock();

        let ttbs = uat.ttbs();

        ttbs[0].ttb0.store(0, Ordering::Relaxed);
        ttbs[0]
            .ttb1
            .store(uat.pagetables_rgn.base | TTBR_VALID, Ordering::Relaxed);

        for ctx in &ttbs[1..] {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        uat.handoff().unlock();

        uat.kpt0()[2].store(ttb1 | PTE_TABLE, Ordering::Relaxed);

        dev_info!(dev, "MMU: initialized\n");

        Ok(uat)
    }
}

impl Drop for Uat {
    fn drop(&mut self) {
        // Disable all contexts. Don't bother locking, since
        // something might have gone horribly wrong with the firmware.
        for ctx in self.ttbs() {
            ctx.ttb0.store(0, Ordering::Relaxed);
            ctx.ttb1.store(0, Ordering::Relaxed);
        }

        // Unmap what we mapped
        self.kpt0()[2].store(0, Ordering::Relaxed);

        // Make sure we flush the TLBs
        fence(Ordering::SeqCst);
        unsafe {
            asm!(".arch armv8.4-a\ntlbi vmalle1os");
        }
    }
}

// SAFETY: All public operations are thread-safe
unsafe impl Send for Uat {}
unsafe impl Sync for Uat {}
