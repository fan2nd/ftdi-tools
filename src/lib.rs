//! This is an [embedded-hal] implementation for the FTDI chips
//!
//! This enables development of embedded device drivers without the use of
//! a microcontroller. The FTDI devices interface with a PC via USB, and
//! provide a multi-protocol synchronous serial engine to interface
//! with most GPIO, SPI, and I2C embedded devices.
//!
//! **Note:**
//! This is strictly a development tool.
//! The crate contains runtime borrow checks and explicit panics to adapt the
//! FTDI device into the [embedded-hal] traits.
//!
//! # Quickstart
//!
//! * Linux users only: Add [udev rules].
//!
//! # Limitations
//!
//! * Limited trait support: SPI, I2C, InputPin, and OutputPin traits are implemented.
//! * Limited device support: FT232H, FT2232H, FT4232H.
//! * Limited SPI modes support: MODE0, MODE2. According to AN108-2.2.

#![forbid(unsafe_code)]

mod ftdaye;
mod gpio;
mod i2c;
mod jtag;
mod list;
mod mpsse_cmd;
mod spi;
mod swd;

pub use ftdaye::Interface;
use ftdaye::{ChipType, FtdiContext, FtdiError};
pub use gpio::{FtdiInputPin, FtdiOutputPin};
pub use i2c::FtdiI2c;
pub use jtag::{
    hw_jtag::FtdiJtag,
    jtag_detect::{JtagDetectTdi, JtagDetectTdo},
};
pub use list::list_all_device;
use mpsse_cmd::MpsseCmdBuilder;
pub use spi::{FtdiSpi, FtdiSpiHalfduplex, SpiMode};
pub use swd::{FtdiSwd, SwdAddr};

/// Pin number
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Pin {
    Lower(usize),
    Upper(usize),
}
impl Pin {
    fn mask(&self) -> u8 {
        match self {
            Pin::Lower(idx) => 1 << idx,
            Pin::Upper(idx) => 1 << idx,
        }
    }
}
/// State tracker for each pin on the FTDI chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PinUse {
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
struct GpioByte {
    /// Direction mask (0 = input, 1 = output) for each pin in the bank
    direction: u8,
    /// Current logic level (0 = low, 1 = high) for each output pin
    value: u8,
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
    lower: GpioByte,
    /// Upper GPIO pins state tracker (if supported by chip)
    upper: GpioByte,
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
    pub fn open(
        usb_device: &nusb::DeviceInfo,
        interface: Interface,
        mask: u8,
    ) -> Result<Self, FtdiError> {
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

        let context = FtdiContext::new(handle, interface, max_packet_size).into_mpsse(mask)?;
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
    /// Supports frequencies from 91Hz to chip-specific maximum (typically 30MHz)
    pub fn set_frequency(&self, frequency_hz: usize) -> Result<usize, FtdiError> {
        const MIN_FREQUENCY: usize = 6_000_000 / (u16::MAX as usize + 1);
        let mut max_frequency = self.chip_type.max_frequency();
        if frequency_hz > max_frequency || frequency_hz < 91 {
            log::error!("speed has out of range[{MIN_FREQUENCY}-{max_frequency}Hz]",)
        }
        let mut cmd = MpsseCmdBuilder::new();
        let mut divide = max_frequency / frequency_hz;
        let mut divide_by_5 = if self.chip_type.has_devide_by5() {
            Some(false)
        } else {
            None
        };
        divide += if max_frequency % frequency_hz != 0 {
            1
        } else {
            0
        };

        if divide - 1 > u16::MAX as usize {
            if divide_by_5.is_some() {
                divide_by_5 = Some(true);
                max_frequency /= 5;
                divide = if divide % 5 != 0 {
                    divide / 5 + 1
                } else {
                    divide / 5
                };
            } else {
                divide_by_5 = None
            }
            if divide - 1 > u16::MAX as usize {
                divide = u16::MAX as usize + 1;
            }
        }
        cmd.set_clock((divide - 1) as u16, divide_by_5);
        self.write_read(cmd.as_slice(), &mut [])?;
        log::info!("Frequency set to {}Hz", max_frequency / divide);
        Ok(max_frequency / divide)
    }
    /// Write mpsse command and read response
    fn write_read(&self, write: &[u8], read: &mut [u8]) -> Result<(), FtdiError> {
        self.ft.write_read(write, read)
    }
    /// Allocate a pin for a specific use.
    fn alloc_pin(&mut self, pin: Pin, purpose: PinUse) {
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
    fn free_pin(&mut self, pin: Pin) {
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
