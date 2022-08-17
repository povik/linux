// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

use kernel::{
    bindings, device,
    error::{to_result, Result},
    io_mem::IoMem,
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::rtkit,
    sync::smutex::Mutex,
    sync::{Arc, ArcBorrow},
};

const ASC_CTL_SIZE: usize = 0x4000;
const CPU_CONTROL: usize = 0x44;
const CPU_RUN: u32 = 0x1 << 4; // BIT(4)

struct AsahiData {
    dev: device::Device,
    rtkit: Mutex<Option<rtkit::RTKit<AsahiDevice>>>,
}

struct AsahiResources {
    asc: IoMem<ASC_CTL_SIZE>,
}

type DeviceData = device::Data<(), AsahiResources, AsahiData>;

struct AsahiDevice;

impl AsahiDevice {
    fn start_cpu(data: ArcBorrow<'_, DeviceData>) -> Result {
        let res = data.resources().ok_or(ENXIO)?;
        let val = res.asc.readl_relaxed(CPU_CONTROL);

        res.asc.writel_relaxed(val | CPU_RUN, CPU_CONTROL);

        Ok(())
    }
}

#[vtable]
impl rtkit::Operations for AsahiDevice {
    type Data = ();
}

extern "C" {
    pub fn asahi_mmu_init(dev: *mut bindings::device) -> core::ffi::c_int;
}

impl platform::Driver for AsahiDevice {
    type Data = Arc<DeviceData>;

    kernel::define_of_id_table! {(), [
        (of::DeviceId::Compatible(b"apple,agx-t8103"), None),
    ]}

    fn probe(
        pdev: &mut platform::Device,
        _id_info: Option<&Self::IdInfo>,
    ) -> Result<Arc<DeviceData>> {
        let dev = device::Device::from_dev(pdev);

        dev_info!(dev, "probing!\n");

        // TODO: add device abstraction to ioremap by name
        // SAFETY: AGX does DMA via the UAT IOMMU (mostly)
        let asc_reg = unsafe { pdev.ioremap_resource(0)? };

        let data = kernel::new_device_data!(
            (),
            AsahiResources {
                // SAFETY: This device does DMA via the UAT IOMMU.
                asc: asc_reg,
            },
            AsahiData {
                dev: dev,
                rtkit: Mutex::new(None),
            },
            "Asahi::Registrations"
        )?;

        let data = Arc::<DeviceData>::from(data);

        AsahiDevice::start_cpu(data.as_ref_borrow())?;

        to_result(unsafe { asahi_mmu_init((&data.dev as &dyn device::RawDevice).raw_device()) })?;

        let mut rtkit = unsafe { rtkit::RTKit::<AsahiDevice>::new(&data.dev, None, 0, ()) }?;

        rtkit.boot()?;
        *data.rtkit.lock() = Some(rtkit);

        dev_info!(data.dev, "probed!\n");
        Ok(data)
    }
}

module_platform_driver! {
    type: AsahiDevice,
    name: "asahi",
    license: "Dual MIT/GPL",
}
