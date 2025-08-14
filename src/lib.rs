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
pub use ftdaye::Interface;
pub mod gpio;
pub mod i2c;
pub mod jtag;
mod list;
pub mod mpsse;
mod mpsse_cmd;
pub mod spi;
pub mod swd;

pub use list::list_all_device;
