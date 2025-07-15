use nusb::DeviceInfo;

use crate::{Interface, ftdaye::ChipType};
/// Known properties associated to particular FTDI chip types.

#[derive(Debug, Clone, Copy)]
struct FtdiDevice {
    /// The (VID, PID) pair of this device.
    id: (u16, u16),

    /// FTDI chip type to use if the device is not recognized.
    ///
    /// "FTDI compatible" devices may use the same VID/PID pair as an FTDI device, but
    /// they may be implemented by a completely third party solution. In this case,
    /// we still try the same `bcdDevice` based detection, but if it fails, we fall back
    /// to this chip type.
    fallback_chip_type: ChipType,
}

/// Known FTDI device variants.
static FTDI_COMPAT_DEVICES: &[FtdiDevice] = &[
    //
    // --- FTDI VID/PID pairs ---
    //
    // FTDI Ltd. FT2232C/D/H Dual UART/FIFO IC
    FtdiDevice {
        id: (0x0403, 0x6010),
        fallback_chip_type: ChipType::FT2232H,
    },
    // FTDI Ltd. FT4232H Quad HS USB-UART/FIFO IC
    FtdiDevice {
        id: (0x0403, 0x6011),
        fallback_chip_type: ChipType::FT4232H,
    },
    // FTDI Ltd. FT232H Single HS USB-UART/FIFO IC
    FtdiDevice {
        id: (0x0403, 0x6014),
        fallback_chip_type: ChipType::FT232H,
    },
    //
    // --- Third-party VID/PID pairs ---
    //
    // Olimex Ltd. ARM-USB-OCD
    FtdiDevice {
        id: (0x15ba, 0x0003),
        fallback_chip_type: ChipType::FT2232C,
    },
    // Olimex Ltd. ARM-USB-TINY
    FtdiDevice {
        id: (0x15ba, 0x0004),
        fallback_chip_type: ChipType::FT2232C,
    },
    // Olimex Ltd. ARM-USB-TINY-H
    FtdiDevice {
        id: (0x15ba, 0x002a),
        fallback_chip_type: ChipType::FT2232H,
    },
    // Olimex Ltd. ARM-USB-OCD-H
    FtdiDevice {
        id: (0x15ba, 0x002b),
        fallback_chip_type: ChipType::FT2232H,
    },
];

pub struct FtdiDeviceInfo {
    pub usb_device: DeviceInfo,
    pub interface: &'static [Interface],
}

pub fn list_all_device() -> Vec<FtdiDeviceInfo> {
    fn filter_map(info: DeviceInfo) -> Option<FtdiDeviceInfo> {
        for device in FTDI_COMPAT_DEVICES {
            if (info.vendor_id(), info.product_id()) == device.id {
                log::info!(
                    "Find {:?}:[{:#06x?},{:#06x?}]",
                    device.fallback_chip_type,
                    device.id.0,
                    device.id.1
                );
                return Some(FtdiDeviceInfo {
                    usb_device: info,
                    interface: device.fallback_chip_type.mpsse_list(),
                });
            }
        }
        None
    }
    nusb::list_devices()
        .unwrap()
        .filter_map(filter_map)
        .collect()
}
