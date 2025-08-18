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
    pub(crate) chip_type: ChipType,
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
            (0x400, _) | (0x200, "") => return Err(FtdiError::UnsupportedChip(ChipType::Bm)),
            (0x200, _) => return Err(FtdiError::UnsupportedChip(ChipType::Am)),
            (0x500, _) => ChipType::FT2232D,
            (0x600, _) => return Err(FtdiError::UnsupportedChip(ChipType::R)),
            (0x700, _) => ChipType::FT2232H,
            (0x800, _) => ChipType::FT4232H,
            (0x900, _) => ChipType::FT232H,
            (0x1000, _) => return Err(FtdiError::UnsupportedChip(ChipType::FT230X)),
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
            .enable_loopback(false);
        if chip_type == ChipType::FT2232D {
            cmd.set_clock(0, None);
        } else {
            cmd.enable_3phase_data_clocking(false)
                .enable_adaptive_clocking(false)
                .set_clock(0, Some(false));
        }
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
    /// FT2232D Supports frequencies from 92Hz to 6MHz.
    /// FTx232H Supports frequencies from 92Hz to 30MHz but in this lib only 458Hz to 30MHz has been supported.
    pub fn set_frequency(&self, frequency_hz: usize) -> Result<usize, FtdiError> {
        let (max_frequency, clk_div_by5) = self.chip_type.max_frequecny();
        let min_frequency = max_frequency / (u16::MAX as usize + 1) + 1;

        let divisor = if frequency_hz > max_frequency {
            log::warn!("frequency has out of range[{min_frequency}-{max_frequency}Hz]");
            log::warn!("frequency set to {max_frequency}Hz]");
            1
        } else if frequency_hz < min_frequency {
            log::warn!("frequency has out of range[{min_frequency}-{max_frequency}Hz]");
            log::warn!("frequency set to {min_frequency}Hz]");
            u16::MAX as usize + 1
        } else if max_frequency % frequency_hz != 0 {
            max_frequency / frequency_hz + 1
        } else {
            max_frequency / frequency_hz
        };

        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_clock((divisor - 1) as u16, clk_div_by5);
        self.exec(cmd)?;
        log::info!("Frequency set to {}Hz", max_frequency / divisor);
        Ok(max_frequency / divisor)
    }
    /// Write mpsse command and read response
    pub(crate) fn exec(&self, cmd: impl Into<MpsseCmdBuilder>) -> Result<Vec<u8>, FtdiError> {
        let mut cmd: MpsseCmdBuilder = cmd.into();
        let mut response = vec![0; cmd.read_len()];
        self.ft.write_read(cmd.as_slice(), &mut response)?;
        Ok(response)
    }
    /// Allocate a pin for a specific use.
    pub(crate) fn alloc_pin(&mut self, pin: Pin, usage: PinUse) -> Result<(), FtdiError> {
        log::trace!("alloc pin {:?} for {:?}", pin, usage);
        if !self.chip_type.mpsse_list().contains(&self.interface)
            && (usage != PinUse::Input || usage != PinUse::Output)
        {
            return Err(FtdiError::IncorrectUsage {
                chip: self.chip_type,
                interface: self.interface,
                usage,
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
                purpose: usage,
                current,
            });
        } else {
            byte.pins[idx] = Some(usage)
        }
        Ok(())
    }
    /// Allocate a pin for a specific use.
    pub(crate) fn free_pin(&mut self, pin: Pin) {
        log::trace!("free pin {:?}", pin);
        match pin {
            Pin::Lower(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                self.lower.pins[idx] = None;
                self.lower.value &= !pin.mask(); // set value to low
                self.lower.direction &= !pin.mask(); // set direction to input
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_lower(self.lower.value, self.lower.direction);
                self.exec(cmd).unwrap();
            }
            Pin::Upper(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                self.upper.pins[idx] = None;
                self.upper.value &= !pin.mask(); // set value to low
                self.upper.direction &= !pin.mask(); // set direction to input
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_upper(self.upper.value, self.upper.direction);
                self.exec(cmd).unwrap();
            }
        };
    }
}
