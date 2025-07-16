use crate::ftdaye::FtdiError;
use crate::mpsse_cmd::MpsseCmdBuilder;
use crate::{FtdiMpsse, Pin, PinUse};
use std::sync::{Arc, Mutex};

/// FTDI GPIO output pin abstraction
///
/// Represents a single GPIO pin configured as output. Manages pin state and
/// ensures proper cleanup through Drop implementation.
pub struct OutputPin {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// GPIO pin identifier
    pin: Pin,
}

impl Drop for OutputPin {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(self.pin);
    }
}

impl OutputPin {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, pin: Pin) -> Result<OutputPin, FtdiError> {
        let mut lock = mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        lock.alloc_pin(pin, PinUse::Input);
        match pin {
            Pin::Lower(idx) => {
                lock.lower.direction |= 1 << idx;
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            Pin::Upper(idx) => {
                lock.upper.direction |= 1 << idx;
                cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
            }
        }
        lock.write_read(cmd.as_slice(), &mut [])?;
        drop(lock);
        Ok(OutputPin { mtx, pin })
    }

    pub(crate) fn set(&self, state: bool) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        match self.pin {
            Pin::Lower(idx) => {
                if state {
                    lock.lower.value |= 1 << idx;
                } else {
                    lock.lower.value &= !(1 << idx);
                }
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            Pin::Upper(idx) => {
                if state {
                    lock.upper.value |= 1 << idx;
                } else {
                    lock.upper.value &= !(1 << idx);
                }
                cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
            }
        }
        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }
}

impl eh1::digital::Error for FtdiError {
    fn kind(&self) -> eh1::digital::ErrorKind {
        eh1::digital::ErrorKind::Other
    }
}

impl eh1::digital::ErrorType for OutputPin {
    type Error = FtdiError;
}

impl eh1::digital::OutputPin for OutputPin {
    fn set_low(&mut self) -> Result<(), FtdiError> {
        self.set(false)
    }

    fn set_high(&mut self) -> Result<(), FtdiError> {
        self.set(true)
    }
}

/// FTDI GPIO input pin abstraction
///
/// Represents a single GPIO pin configured as input. Provides methods to read
/// pin state and ensures proper cleanup through Drop implementation.
pub struct InputPin {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// GPIO pin index.
    pin: Pin,
}

impl Drop for InputPin {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(self.pin);
    }
}

impl InputPin {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, pin: Pin) -> Result<InputPin, FtdiError> {
        let mut lock = mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        lock.alloc_pin(pin, PinUse::Input);
        match pin {
            Pin::Lower(idx) => {
                lock.lower.direction &= !(1 << idx);
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            Pin::Upper(idx) => {
                lock.upper.direction &= !(1 << idx);
                cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
            }
        }
        lock.write_read(cmd.as_slice(), &mut [])?;
        drop(lock);
        Ok(InputPin { mtx, pin })
    }

    pub(crate) fn get(&self) -> Result<bool, FtdiError> {
        let lock = self.mtx.lock().unwrap();

        let mut response = [0u8];
        let mut cmd = MpsseCmdBuilder::new();
        match self.pin {
            Pin::Lower(_) => cmd.gpio_lower(),
            Pin::Upper(_) => cmd.gpio_upper(),
        };
        lock.write_read(cmd.as_slice(), &mut response)?;

        Ok((response[0] & self.pin.mask()) != 0)
    }
}

impl eh1::digital::ErrorType for InputPin {
    type Error = FtdiError;
}

impl eh1::digital::InputPin for InputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.get()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.get().map(|res| !res)
    }
}
