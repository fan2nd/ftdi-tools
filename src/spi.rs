use crate::{
    FtdiError, Pin,
    gpio::UsedPin,
    mpsse::{FtdiMpsse, PinUse},
    mpsse_cmd::MpsseCmdBuilder,
};
use eh1::spi::{Error, ErrorKind, ErrorType, MODE_0, MODE_2, Mode, Operation, SpiBus, SpiDevice};
use std::sync::{Arc, Mutex};

const SCK_MASK: u8 = Pin::Lower(0).mask();
const MOSI_MASK: u8 = Pin::Lower(1).mask();
#[allow(unused)]
const MISO_MASK: u8 = Pin::Lower(2).mask();
const CS_MASK: u8 = Pin::Lower(3).mask();

// Spi only support mode0 and mode2
// TDI(AD1) can only can output on second edge.
// TDO(AD2) can only can sample on first edge.
// according to AN108-2.2.
// https://ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf

#[derive(Debug, thiserror::Error)]
pub enum FtdiSpiError {
    #[error(transparent)]
    FtdiInner(#[from] FtdiError),
    #[error("embedded-hal::spi::SpiBus {0} is not supported.")]
    NotSupported(&'static str),
}
impl Error for FtdiSpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}
/// FTDI SPI bus.
///
/// In embedded-hal version 1 this represents an exclusive SPI bus.
/// Serial Peripheral Interface (SPI) master controller using FTDI MPSSE
///
/// Implements full-duplex synchronous serial communication with configurable mode
pub struct FtdiSpi {
    _pins: [UsedPin; 3],
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Initial value of SCK line (clock polarity) - determines idle state
    tck_init_value: bool,
    /// Whether data is transferred least significant bit (LSB) first
    is_lsb: bool,
}

impl FtdiSpi {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUse::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };

        let mut lock = mtx.lock().unwrap();
        // default MODE0, SCK(AD0) default 0
        // set SCK(AD0) and MOSI (AD1) as output pins
        lock.lower.direction |= SCK_MASK | MOSI_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;

        // default msb mode0
        Ok(this)
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: Mode, is_lsb: bool) -> Result<(), FtdiSpiError> {
        let mut lock = self.mtx.lock().unwrap();
        // set SCK polarity
        match mode {
            MODE_0 => {
                lock.lower.value &= !SCK_MASK; // set SCK(AD0) to 0
                self.tck_init_value = false;
            }
            MODE_2 => {
                lock.lower.value |= SCK_MASK; // set SCK(AD0) to 1
                self.tck_init_value = true;
            }
            _ => {
                return Err(FtdiSpiError::NotSupported("MODE_1&MODE_3"));
            }
        }
        self.is_lsb = is_lsb;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;
        Ok(())
    }
}

impl ErrorType for FtdiSpi {
    type Error = FtdiSpiError;
}

impl SpiBus<u8> for FtdiSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let lock = self.mtx.lock().unwrap();
        let response = lock.exec(cmd)?;
        words.copy_from_slice(&response);

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes_out(self.tck_init_value, self.is_lsb, words);

        let lock = self.mtx.lock().unwrap();
        lock.exec(cmd)?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes(self.tck_init_value, self.is_lsb, words);

        let lock = self.mtx.lock().unwrap();

        let response = lock.exec(cmd)?;
        words.copy_from_slice(&response);

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes(self.tck_init_value, self.is_lsb, write);

        let lock = self.mtx.lock().unwrap();
        let response = lock.exec(cmd)?;
        read.copy_from_slice(&response);

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
    _pins: [UsedPin; 3],
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Initial value of SCK line (clock polarity) - determines idle state
    tck_init_value: bool,
    /// Whether data is transferred least significant bit (LSB) first
    is_lsb: bool,
}

impl FtdiSpiHalfduplex {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSpiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUse::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };

        let mut lock = mtx.lock().unwrap();
        // default MODE0, SCK(AD0) default 0
        // set SCK(AD0) and MOSI (AD1) as output pins
        lock.lower.direction |= SCK_MASK | MOSI_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;

        // default msb mode0
        Ok(this)
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: Mode, is_lsb: bool) -> Result<(), FtdiSpiError> {
        let mut lock = self.mtx.lock().unwrap();
        // set SCK polarity
        match mode {
            MODE_0 => {
                lock.lower.value &= !SCK_MASK; // set SCK(AD0) to 0
                self.tck_init_value = false;
            }
            MODE_2 => {
                lock.lower.value |= SCK_MASK; // set SCK(AD0) to 1
                self.tck_init_value = true;
            }
            _ => {
                return Err(FtdiSpiError::NotSupported("MODE_1&MODE_3"));
            }
        }
        self.is_lsb = is_lsb;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;
        Ok(())
    }
    /// set spi bps
    pub fn set_frequency(&mut self, bps: usize) -> Result<usize, FtdiSpiError> {
        let lock = self.mtx.lock().unwrap();
        Ok(lock.set_frequency(bps)?)
    }
}

impl ErrorType for FtdiSpiHalfduplex {
    type Error = FtdiSpiError;
}

impl SpiBus for FtdiSpiHalfduplex {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction & (!MOSI_MASK)); // set tdi to input
        cmd.clock_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let response = lock.exec(cmd)?;
        words.copy_from_slice(&response);

        Ok(())
    }
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        cmd.clock_bytes_out(self.tck_init_value, self.is_lsb, words);

        lock.exec(cmd)?;

        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer"))
    }
    fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer_in_place"))
    }
}
pub struct FtdiSpiDevice {
    _pins: [UsedPin; 4],
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Initial value of SCK line (clock polarity) - determines idle state
    tck_init_value: bool,
    /// Whether data is transferred least significant bit (LSB) first
    is_lsb: bool,
}

impl FtdiSpiDevice {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSpiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUse::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(3), PinUse::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };
        let mut lock = mtx.lock().unwrap();
        // default MODE0, SCK(AD0) default 0
        // set SCK(AD0) and MOSI (AD1) as output pins
        lock.lower.direction |= SCK_MASK | MOSI_MASK | CS_MASK;
        lock.lower.value |= CS_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;
        // default msb mode0
        Ok(this)
    }
}

impl ErrorType for FtdiSpiDevice {
    type Error = FtdiSpiError;
}

impl SpiDevice<u8> for FtdiSpiDevice {
    fn transaction(
        &mut self,
        operations: &mut [eh1::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(
            lock.lower.value & !Pin::Lower(3).mask(),
            lock.lower.direction,
        );
        operations.iter().for_each(|op| match op {
            Operation::Read(read) => {
                cmd.clock_bytes_in(self.tck_init_value, self.is_lsb, read.len());
            }
            Operation::Write(write) => {
                cmd.clock_bytes_out(self.tck_init_value, self.is_lsb, write);
            }
            Operation::Transfer(_, write) => {
                cmd.clock_bytes(self.tck_init_value, self.is_lsb, write);
            }
            Operation::TransferInPlace(write) => {
                cmd.clock_bytes(self.tck_init_value, self.is_lsb, write);
            }
            Operation::DelayNs(_) => (),
        });
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        let response = lock.exec(cmd)?;
        let mut len = 0;
        operations.iter_mut().for_each(|op| {
            len += match op {
                Operation::Read(x) => {
                    x.copy_from_slice(&response[len..x.len()]);
                    x.len()
                }
                Operation::Transfer(x, _) => {
                    x.copy_from_slice(&response[len..x.len()]);
                    x.len()
                }
                Operation::TransferInPlace(x) => {
                    x.copy_from_slice(&response[len..x.len()]);
                    x.len()
                }
                _ => 0,
            }
        });
        Ok(())
    }
}
