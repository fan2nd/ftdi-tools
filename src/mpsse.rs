use crate::{ChipType, FtdiError, Interface, Pin, ftdaye::FtdiContext, mpsse_cmd::MpsseCmdBuilder};
/// State tracker for each pin on the FTDI chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinUse {
    Output,
    Input,
    I2c,
    Spi,
    Jtag,
    Swd,
}
/// Manages a bank of 8 GPIO pins
/// Tracks direction, current value, and allocated protocol usage
#[derive(Debug, Default)]
pub(crate) struct GpioByte {
    /// Direction mask (0 = input, 1 = output) for each pin in the bank
    pub(crate) direction: u8,
    /// Current logic level (0 = low, 1 = high) for each output pin
    pub(crate) value: u8,
    /// Protocol allocation status for each pin (prevents conflicting usage)
    pins: [Option<PinUse>; 8],
}

/// Main FTDI MPSSE (Multi-Protocol Synchronous Serial Engine) controller
/// Manages FTDI device communication and protocol-specific pin configurations
pub struct FtdiMpsse {
    /// FTDI device context handle
    ft: FtdiContext,
    /// FTDI device interface
    interface: Interface,
    /// Type of FTDI chip (e.g., FT232H, FT2232H)
    chip_type: ChipType,
    /// Lower 8 GPIO pins state tracker
    pub(crate) lower: GpioByte,
    /// Upper GPIO pins state tracker (if supported by chip)
    pub(crate) upper: GpioByte,
}

impl FtdiMpsse {
    /// Opens and initializes an FTDI device in MPSSE mode
    ///
    /// # Arguments
    /// * `usb_device` - USB device information from enumeration
    /// * `interface` - FTDI interface to use (A, B, etc.)
    /// * `mask` - Initial GPIO pin direction mask
    ///
    /// # Returns
    /// Result containing FtdiMpsse instance or FtdiError
    pub fn open(usb_device: &nusb::DeviceInfo, interface: Interface) -> Result<Self, FtdiError> {
        let handle = usb_device.open()?;
        let max_packet_size = handle
            .active_configuration()
            .map_err(|e| FtdiError::Usb(e.into()))?
            .interface_alt_settings()
            .next()
            .ok_or(FtdiError::OpenFailed(
                "Failed to get interface info".to_string(),
            ))?
            .endpoints()
            .next()
            .ok_or(FtdiError::OpenFailed(
                "Failed to get endpoint info".to_string(),
            ))?
            .max_packet_size();
        let chip_type = match (
            usb_device.device_version(),
            usb_device.serial_number().unwrap_or(""),
        ) {
            (0x400, _) | (0x200, "") => return Err(FtdiError::UnsupportedChipType(ChipType::Bm)),
            (0x200, _) => return Err(FtdiError::UnsupportedChipType(ChipType::Am)),
            (0x500, _) => return Err(FtdiError::UnsupportedChipType(ChipType::FT2232C)),
            (0x600, _) => return Err(FtdiError::UnsupportedChipType(ChipType::R)),
            (0x700, _) => ChipType::FT2232H,
            (0x800, _) => ChipType::FT4232H,
            (0x900, _) => ChipType::FT232H,
            (0x1000, _) => return Err(FtdiError::UnsupportedChipType(ChipType::FT230X)),
            (version, _) => {
                return Err(FtdiError::OpenFailed(format!(
                    "Unknown ChipType version:0x{version:x}"
                )));
            }
        };
        if !chip_type.interface_list().contains(&interface) {
            return Err(FtdiError::OpenFailed(format!(
                "{chip_type:?} do not support Interface::{interface:?}"
            )));
        }

        let handle = handle.detach_and_claim_interface(interface.interface_number())?;

        let context = FtdiContext::new(handle, interface, max_packet_size).into_mpsse(0)?;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(0, 0) // set all pin to input and value 0;
            .set_gpio_upper(0, 0) // set all pin to input and value 0;
            .enable_loopback(false)
            .enable_3phase_data_clocking(false)
            .enable_adaptive_clocking(false)
            .set_clock(0, Some(false));
        context.write_read(cmd.as_slice(), &mut [])?;

        Ok(Self {
            ft: context,
            interface,
            chip_type,
            lower: Default::default(),
            upper: Default::default(),
        })
    }

    /// Sets the MPSSE clock frequency
    ///
    /// # Arguments
    /// * `frequency_hz` - Target frequency in Hertz
    ///
    /// # Returns
    /// Result containing the actual set frequency or FtdiError
    ///
    /// # Notes
    /// Actual frequency may differ from target due to hardware limitations
    /// Supports frequencies from 458Hz to chip-specific maximum (typically 30MHz)
    pub fn set_frequency(&self, frequency_hz: usize) -> Result<usize, FtdiError> {
        const MAX_FREQUENCY: usize = 30_000_000;
        const MIN_FREQUENCY: usize = MAX_FREQUENCY / (u16::MAX as usize + 1) + 1;

        let divisor = if frequency_hz > MAX_FREQUENCY {
            log::warn!("frequency has out of range[{MIN_FREQUENCY}-{MAX_FREQUENCY}Hz]");
            1
        } else if frequency_hz < MIN_FREQUENCY {
            log::warn!("frequency has out of range[{MIN_FREQUENCY}-{MAX_FREQUENCY}Hz]");
            u16::MAX as usize + 1
        } else if MAX_FREQUENCY % frequency_hz != 0 {
            MAX_FREQUENCY / frequency_hz + 1
        } else {
            MAX_FREQUENCY / frequency_hz
        };

        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_clock((divisor - 1) as u16, Some(false)); // [`EnableClockDivideBy5`] is useless because 458hz is already slow.
        self.write_read(cmd.as_slice(), &mut [])?;
        log::info!("Frequency set to {}Hz", MAX_FREQUENCY / divisor);
        Ok(MAX_FREQUENCY / divisor)
    }
    /// Write mpsse command and read response
    pub(crate) fn write_read(&self, write: &[u8], read: &mut [u8]) -> Result<(), FtdiError> {
        self.ft.write_read(write, read)
    }
    /// Allocate a pin for a specific use.
    pub(crate) fn alloc_pin(&mut self, pin: Pin, purpose: PinUse) -> Result<(), FtdiError> {
        if !self.chip_type.mpsse_list().contains(&self.interface)
            && (purpose != PinUse::Input || purpose != PinUse::Output)
        {
            return Err(FtdiError::IncorrectUsage {
                chip: self.chip_type,
                interface: self.interface,
                usage: purpose,
            });
        };
        let (byte, idx) = match pin {
            Pin::Lower(idx) => {
                if idx >= 8 {
                    return Err(FtdiError::PinNotVaild {
                        chip: self.chip_type,
                        interface: self.interface,
                        pin,
                    });
                };
                (&mut self.lower, idx)
            }
            Pin::Upper(idx) => {
                if idx >= self.chip_type.upper_pins() {
                    return Err(FtdiError::PinNotVaild {
                        chip: self.chip_type,
                        interface: self.interface,
                        pin,
                    });
                }
                (&mut self.upper, idx)
            }
        };
        if let Some(current) = byte.pins[idx] {
            return Err(FtdiError::PinInUsed {
                pin,
                purpose,
                current,
            });
        } else {
            byte.pins[idx] = Some(purpose)
        }
        Ok(())
    }
    /// Allocate a pin for a specific use.
    pub(crate) fn free_pin(&mut self, pin: Pin) {
        match pin {
            Pin::Lower(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                self.lower.pins[idx] = None;
                self.lower.value &= !(1 << idx); // set value to low
                self.lower.direction &= !(1 << idx); // set direction to input
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_lower(self.lower.value, self.lower.direction);
                self.write_read(cmd.as_slice(), &mut []).unwrap();
            }
            Pin::Upper(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                self.upper.pins[idx] = None;
                self.upper.value &= !(1 << idx); // set value to low
                self.upper.direction &= !(1 << idx); // set direction to input
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_upper(self.upper.value, self.upper.direction);
                self.write_read(cmd.as_slice(), &mut []).unwrap();
            }
        };
    }
}
