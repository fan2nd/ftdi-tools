use crate::{
    ftdaye::{ChipType, FtdiContext, FtdiError, Interface},
    mpsse_cmd::MpsseCmdBuilder,
};
/// Pin number
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Pin {
    Lower(usize),
    Upper(usize),
}
impl Pin {
    pub(crate) fn mask(&self) -> u8 {
        match self {
            Pin::Lower(idx) => 1 << idx,
            Pin::Upper(idx) => 1 << idx,
        }
    }
}
/// State tracker for each pin on the FTDI chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PinUse {
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
        let max_packet_size = {
            let interface_alt_settings: Vec<_> = handle
                .active_configuration()?
                .interface_alt_settings()
                .collect();
            let endpoints: Vec<_> = interface_alt_settings[interface as usize - 1]
                .endpoints()
                .collect();
            endpoints[0].max_packet_size()
        };
        let chip_type = match (
            usb_device.device_version(),
            usb_device.serial_number().unwrap_or(""),
        ) {
            // (0x400, _) | (0x200, "") => ChipType::Bm,
            // (0x200, _) => ChipType::Am,
            // (0x500, _) => ChipType::FT2232C,
            // (0x600, _) => ChipType::R,
            (0x700, _) => ChipType::FT2232H,
            (0x800, _) => ChipType::FT4232H,
            (0x900, _) => ChipType::FT232H,
            // (0x1000, _) => ChipType::FT230X,
            (version, _) => {
                return Err(FtdiError::Other(format!(
                    "Unknown ChipType version:0x{version:x}"
                )));
            }
        };
        if !chip_type.mpsse_list().contains(&interface) {
            return Err(FtdiError::Other(format!(
                "{chip_type:?} do not has {interface:?}"
            )));
        }

        let handle = handle.detach_and_claim_interface(interface as u8 - 1)?;

        let context = FtdiContext::new(handle, interface, max_packet_size).into_mpsse(0)?;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(0, 0) // set all pin to input and value 0;
            .set_gpio_upper(0, 0) // set all pin to input and value 0;
            .enable_loopback(false)
            .enable_3phase_data_clocking(false)
            .enable_adaptive_clocking(false);
        context.write_read(cmd.as_slice(), &mut [])?;

        Ok(Self {
            ft: context,
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
    pub(crate) fn alloc_pin(&mut self, pin: Pin, purpose: PinUse) {
        let (byte, idx) = match pin {
            Pin::Lower(idx) => (&mut self.lower, idx),
            Pin::Upper(idx) => {
                assert!(
                    idx < self.chip_type.upper_pins(),
                    "{:?} do not has {:?}",
                    self.chip_type,
                    pin
                );
                (&mut self.upper, idx)
            }
        };
        assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
        if let Some(current) = byte.pins[idx] {
            panic!(
                "Unable to allocate pin {pin:?} for {purpose:?}, pin is already allocated for {current:?}"
            );
        } else {
            byte.pins[idx] = Some(purpose)
        }
    }
    /// Allocate a pin for a specific use.
    pub(crate) fn free_pin(&mut self, pin: Pin) {
        match pin {
            Pin::Lower(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                assert!(self.lower.pins[idx].is_some(), "{pin:?} is not used");
                self.lower.pins[idx] = None;
                self.lower.value &= !(1 << idx); // set value to low
                self.lower.direction &= !(1 << idx); // set direction to input
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_lower(self.lower.value, self.lower.direction);
                self.write_read(cmd.as_slice(), &mut []).unwrap();
            }
            Pin::Upper(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                assert!(self.upper.pins[idx].is_some(), "{pin:?} is not used");
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
