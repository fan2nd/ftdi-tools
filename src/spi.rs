use crate::ftdaye::FtdiError;
use crate::mpsse_cmd::MpsseCmdBuilder;
use crate::{FtdiMpsse, Pin, PinUse};
use eh1::spi::{Error, ErrorKind, ErrorType, SpiBus};
use std::sync::{Arc, Mutex};

const SCK_MASK: u8 = 1 << 0;
const MOSI_MASK: u8 = 1 << 1;
#[allow(unused)]
const MISO_MASK: u8 = 1 << 2;

// Spi only support mode0 and mode2
// TDI(AD1) can only can output on second edge.
// TDO(AD2) can only can sample on first edge.
// according to AN108-2.2.
// https://ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
#[derive(Debug, Clone, Copy)]
pub enum SpiMode {
    MsbMode0,
    LsbMode0,
    MsbMode2,
    LsbMode2,
}

/// FTDI SPI bus.
///
/// In embedded-hal version 1 this represents an exclusive SPI bus.
/// Serial Peripheral Interface (SPI) master controller using FTDI MPSSE
///
/// Implements full-duplex synchronous serial communication with configurable mode
pub struct FtdiSpi {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Initial value of SCK line (clock polarity) - determines idle state
    tck_init_value: bool,
    /// Whether data is transferred least significant bit (LSB) first
    is_lsb: bool,
}

impl Drop for FtdiSpi {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl FtdiSpi {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<FtdiSpi, FtdiError> {
        {
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(Pin::Lower(0), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(1), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(2), PinUse::Spi);

            // default MODE0, SCK(AD0) default 0
            // set SCK(AD0) and MOSI (AD1) as output pins
            lock.lower.direction |= SCK_MASK | MOSI_MASK;

            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        // default msb mode0
        Ok(FtdiSpi {
            mtx,
            tck_init_value: false,
            is_lsb: false,
        })
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: SpiMode) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();
        // set SCK polarity
        match mode {
            SpiMode::MsbMode0 | SpiMode::LsbMode0 => lock.lower.value &= !(0x01), // set SCK(AD0) to 0
            SpiMode::MsbMode2 | SpiMode::LsbMode2 => lock.lower.value |= 0x01, // set SCK(AD0) to 1
        }
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.write_read(cmd.as_slice(), &mut [])?;
        (self.tck_init_value, self.is_lsb) = match mode {
            SpiMode::MsbMode0 => (false, false),
            SpiMode::LsbMode0 => (false, true),
            SpiMode::MsbMode2 => (true, false),
            SpiMode::LsbMode2 => (true, true),
        };
        Ok(())
    }
}

impl Error for FtdiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for FtdiSpi {
    type Error = FtdiError;
}

impl SpiBus<u8> for FtdiSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes_out(self.tck_init_value, self.is_lsb, words);

        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes(self.tck_init_value, self.is_lsb, words);

        let lock = self.mtx.lock().unwrap();

        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes(self.tck_init_value, self.is_lsb, write);

        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), read)?;

        Ok(())
    }
}

/// FTDI SPI bus.
///
/// In embedded-hal version 1 this represents an exclusive SPI bus.
/// Serial Peripheral Interface (SPI) master controller using FTDI MPSSE
///
/// Implements full-duplex synchronous serial communication with configurable mode
pub struct FtdiSpiHalfduplex {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Initial value of SCK line (clock polarity) - determines idle state
    tck_init_value: bool,
    /// Whether data is transferred least significant bit (LSB) first
    is_lsb: bool,
}

impl Drop for FtdiSpiHalfduplex {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl FtdiSpiHalfduplex {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<FtdiSpiHalfduplex, FtdiError> {
        {
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(Pin::Lower(0), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(1), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(2), PinUse::Spi);

            // default MODE0, SCK(AD0) default 0
            // set SCK(AD0) and MOSI (AD1) as output pins
            lock.lower.direction |= 0x03;

            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(FtdiSpiHalfduplex {
            mtx,
            tck_init_value: false,
            is_lsb: false,
        })
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: SpiMode) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();
        // set SCK polarity
        match mode {
            SpiMode::MsbMode0 | SpiMode::LsbMode0 => lock.lower.value &= !(0x01), // set SCK(AD0) to 0
            SpiMode::MsbMode2 | SpiMode::LsbMode2 => lock.lower.value |= 0x01, // set SCK(AD0) to 1
        }
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.write_read(cmd.as_slice(), &mut [])?;
        (self.tck_init_value, self.is_lsb) = match mode {
            SpiMode::MsbMode0 => (false, false),
            SpiMode::LsbMode0 => (false, true),
            SpiMode::MsbMode2 => (true, false),
            SpiMode::LsbMode2 => (true, true),
        };
        Ok(())
    }
}

impl ErrorType for FtdiSpiHalfduplex {
    type Error = FtdiError;
}

impl SpiBus for FtdiSpiHalfduplex {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction & (!MOSI_MASK)); // set tdi to input
        cmd.clock_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        cmd.clock_bytes_out(self.tck_init_value, self.is_lsb, words);

        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        panic!("Not supported")
    }
    fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        panic!("Not supported")
    }
}
