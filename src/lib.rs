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

pub mod delay;
mod ftdaye;
pub mod gpio;
pub mod i2c;
pub mod jtag;
mod list;
pub use list::list_all_device;
pub mod mpsse;
mod mpsse_cmd;
pub mod spi;
pub mod swd;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipType {
    Am,
    Bm,
    FT2232D,
    R,
    FT2232H,
    FT4232H,
    FT232H,
    FT230X,
    Unknown,
}
impl ChipType {
    pub(crate) const fn interface_list(self) -> &'static [Interface] {
        match self {
            ChipType::FT232H => &[Interface::A],
            ChipType::FT2232H | ChipType::FT2232D => &[Interface::A, Interface::B],
            ChipType::FT4232H => &[Interface::A, Interface::B, Interface::C, Interface::D],
            _ => &[],
        }
    }
    pub(crate) const fn mpsse_list(self) -> &'static [Interface] {
        match self {
            ChipType::FT232H | ChipType::FT2232D => &[Interface::A],
            ChipType::FT2232H | ChipType::FT4232H => &[Interface::A, Interface::B],
            _ => &[],
        }
    }
    pub(crate) const fn upper_pins(self) -> usize {
        match self {
            ChipType::FT232H | ChipType::FT2232H => 8,
            ChipType::FT2232D => 4,
            ChipType::FT4232H => 0,
            _ => 0,
        }
    }
    pub(crate) const fn max_frequecny(self) -> (usize, Option<bool>) {
        match self {
            ChipType::FT2232D => (6_000_000, None),
            ChipType::FT232H | ChipType::FT2232H | ChipType::FT4232H => (30_000_000, Some(false)),
            _ => (0, None),
        }
    }
    pub(crate) const fn max_packet_size(self) -> usize {
        match self {
            ChipType::FT2232D => 64,
            ChipType::FT232H | ChipType::FT2232H | ChipType::FT4232H => 512,
            _ => 64,
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Interface {
    A = 1,
    B = 2,
    C = 3,
    D = 4,
}

impl Interface {
    pub(crate) const fn read_ep(self) -> u8 {
        match self {
            Interface::A => 0x81,
            Interface::B => 0x83,
            Interface::C => 0x85,
            Interface::D => 0x87,
        }
    }

    pub(crate) const fn write_ep(self) -> u8 {
        match self {
            Interface::A => 0x02,
            Interface::B => 0x04,
            Interface::C => 0x06,
            Interface::D => 0x08,
        }
    }

    pub(crate) const fn index(self) -> u16 {
        self as u16
    }

    pub(crate) const fn interface_number(self) -> u8 {
        (self as u8) - 1
    }
}
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Pin {
    Lower(usize),
    Upper(usize),
}
impl Pin {
    pub(crate) const fn mask(self) -> u8 {
        match self {
            Pin::Lower(idx) => 1 << idx,
            Pin::Upper(idx) => 1 << idx,
        }
    }
}
#[derive(Debug, thiserror::Error)]
pub enum FtdiError {
    #[error("A USB transport error occurred.")]
    Usb(#[from] std::io::Error),

    #[error("Open failed: {0}")]
    /// Error occurs when open.
    OpenFailed(String),

    #[error("Unsupported chip type: {0:?}")]
    /// The connected device is not supported by the driver.
    UnsupportedChip(ChipType),

    #[error("Bad Mpsse Command: {0:#x}")]
    BadMpsseCommand(u8),

    #[error("Pin fault: {0}")]
    PinFault(String),

    #[error("{0}")]
    Other(&'static str),
}
