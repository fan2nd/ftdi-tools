use crate::ftdaye::FtdiError;
use crate::i2c::cmd::I2cCmdBuilder;
use crate::mpsse_cmd::MpsseCmdBuilder;
use crate::{FtdiMpsse, Pin, PinUse};
use eh1::i2c::{ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};
use std::sync::{Arc, Mutex};

// because i2c is msb
const SLAVE_ACK_MASK: u8 = 1 << 0;

/// Inter-Integrated Circuit (I2C) master controller using FTDI MPSSE
///
/// Implements I2C bus communication with support for start/stop conditions and clock stretching
pub struct I2c {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Length of start, repeated start, and stop conditions in MPSSE commands
    /// More commands increase the duration of these conditions
    start_stop_cmds: usize,
    /// Optional direction pin for SDA line direction control (if used)
    direction_pin: Option<Pin>,
}

impl Drop for I2c {
    fn drop(&mut self) {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.enable_3phase_data_clocking(false);
        let mut lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut []).unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
        if let Some(pin) = self.direction_pin {
            lock.free_pin(pin);
        }
    }
}

impl I2c {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<I2c, FtdiError> {
        {
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(Pin::Lower(0), PinUse::I2c);
            lock.alloc_pin(Pin::Lower(1), PinUse::I2c);
            lock.alloc_pin(Pin::Lower(2), PinUse::I2c);
            // clear direction and value of first 3 pins
            // pins default input and value 0
            // AD0: SCL
            // AD1: SDA (master out)
            // AD2: SDA (master in)
            let mut cmd = MpsseCmdBuilder::new();
            cmd.enable_3phase_data_clocking(true);
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        let this = I2c {
            mtx,
            start_stop_cmds: 3,
            direction_pin: None,
        };
        log::info!("IIC default 100Khz");
        this.set_frequency(100_000)?;
        Ok(this)
    }

    pub fn set_direction_pin(&mut self, pin: Pin) {
        let mut lock = self.mtx.lock().unwrap();
        if let Some(pin) = self.direction_pin {
            lock.free_pin(pin);
        }
        lock.alloc_pin(pin, PinUse::Swd);
        match pin {
            Pin::Lower(idx) => {
                lock.lower.direction |= 1 << idx;
            }
            Pin::Upper(idx) => {
                lock.upper.direction |= 1 << idx;
            }
        }
        self.direction_pin = Some(pin)
    }

    /// Set the length of start and stop conditions.
    ///
    /// This is an advanced feature that most people will not need to touch.
    /// I2C start and stop conditions are generated with a number of MPSSE
    /// commands.  This sets the number of MPSSE command generated for each
    /// stop and start condition.  An increase in the number of MPSSE commands
    /// roughtly correlates to an increase in the duration.
    pub fn set_stop_start_len(&mut self, start_stop_cmds: usize) {
        self.start_stop_cmds = start_stop_cmds
    }

    pub fn set_frequency(&self, frequency_hz: usize) -> Result<(), FtdiError> {
        let lock = self.mtx.lock().unwrap();
        lock.set_frequency(frequency_hz * 3 / 2)?;
        Ok(())
    }

    pub fn scan(&mut self) -> Vec<u8> {
        let mut addr_set = Vec::new();
        for addr in 0..128 {
            let read_buf = &mut [0];
            if self
                .transaction(addr, &mut [Operation::Read(read_buf)])
                .is_ok()
            {
                addr_set.push(addr);
            }
        }
        addr_set
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), ErrorKind> {
        // lock at the start to prevent GPIO from being modified while we build
        // the MPSSE command
        let lock = self.mtx.lock().unwrap();

        // start
        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin);
        cmd.start(self.start_stop_cmds);
        lock.write_read(cmd.as_slice(), &mut [])?;

        let mut prev_op_was_a_read: bool = false;
        for (idx, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if idx == 0 || !prev_op_was_a_read {
                        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin);
                        if idx != 0 {
                            // repeated start
                            cmd.start(self.start_stop_cmds);
                        }
                        // addr + ack_read
                        cmd.i2c_addr(address, true);

                        let mut response = [0];
                        lock.write_read(cmd.as_slice(), &mut response)?;
                        if (response[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }

                    let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin);
                    for idx in 0..buffer.len() {
                        if idx == buffer.len() - 1 {
                            // NMAK: Master Not Ack
                            cmd.i2c_read(false);
                        } else {
                            // MAK: Master Ack
                            cmd.i2c_read(true);
                        }
                    }
                    lock.write_read(cmd.as_slice(), buffer)?;

                    prev_op_was_a_read = true;
                }
                Operation::Write(bytes) => {
                    if idx == 0 || prev_op_was_a_read {
                        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin);
                        if idx != 0 {
                            // repeated start
                            cmd.start(self.start_stop_cmds);
                        }
                        cmd
                            // Address+Write+Ack
                            .i2c_addr(address, false);

                        let mut response = [0u8];
                        lock.write_read(cmd.as_slice(), &mut response)?;
                        if (response[0] & SLAVE_ACK_MASK) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }

                    for &byte in *bytes {
                        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin);
                        cmd.i2c_write(byte);
                        let mut response = [0u8];
                        lock.write_read(cmd.as_slice(), &mut response)?;
                        if (response[0] & SLAVE_ACK_MASK) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                        }
                    }
                    prev_op_was_a_read = false;
                }
            }
        }

        // stop
        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin);
        cmd.end(self.start_stop_cmds);
        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }
}

impl From<FtdiError> for ErrorKind {
    fn from(_value: FtdiError) -> Self {
        ErrorKind::Other
    }
}

impl eh1::i2c::ErrorType for I2c {
    type Error = eh1::i2c::ErrorKind;
}

/// I2C trait implementation for FTDI MPSSE
impl eh1::i2c::I2c for I2c {
    /// Executes a sequence of I2C operations (read/write) on the specified device
    ///
    /// # Arguments
    /// * `address` - 7-bit I2C device address
    /// * `operations` - Slice of read/write operations to perform
    ///
    /// # Returns
    /// Result indicating success or failure of the transaction
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction(address, operations)
    }
}

mod cmd {
    const SCL: u8 = 1 << 0; // SCK bitmask
    const SDA: u8 = 1 << 1; // DIO bitmask
    const TCK_INIT_VALUE: bool = false;
    const IS_LSB: bool = false;
    const DATA_BITS: usize = 8;
    const ACK_BITS: usize = 1;

    use crate::{FtdiMpsse, Pin, mpsse_cmd::MpsseCmdBuilder};
    use std::{
        ops::{Deref, DerefMut},
        sync::MutexGuard,
    };
    pub(super) struct I2cCmdBuilder<'a> {
        cmd: MpsseCmdBuilder,
        lock: &'a MutexGuard<'a, FtdiMpsse>,
        direction_pin: Option<Pin>,
    }
    impl<'a> Deref for I2cCmdBuilder<'a> {
        type Target = MpsseCmdBuilder;
        fn deref(&self) -> &Self::Target {
            &self.cmd
        }
    }
    impl<'a> DerefMut for I2cCmdBuilder<'a> {
        fn deref_mut(&mut self) -> &mut Self::Target {
            &mut self.cmd
        }
    }
    impl<'a> I2cCmdBuilder<'a> {
        pub(super) fn new(lock: &'a MutexGuard<FtdiMpsse>, direction_pin: Option<Pin>) -> Self {
            I2cCmdBuilder {
                cmd: MpsseCmdBuilder::new(),
                lock,
                direction_pin,
            }
        }
        fn i2c_out(&mut self, scl: bool, sda: bool) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            let scl = if scl { SCL } else { 0 };
            let sda = if sda { SDA } else { 0 };
            if let Some(pin) = self.direction_pin {
                match pin {
                    Pin::Lower(idx) => {
                        self.set_gpio_lower(
                            lower_value | (1 << idx) | scl | sda,
                            lower_direction | SCL | SDA,
                        );
                    }
                    Pin::Upper(idx) => {
                        self.set_gpio_lower(lower_value | scl | sda, lower_direction | SCL | SDA);
                        self.set_gpio_upper(upper_value | (1 << idx), upper_direction);
                    }
                }
            } else {
                self.set_gpio_lower(lower_value | scl | sda, lower_direction | SCL | SDA);
            }
            self
        }
        fn i2c_in(&mut self) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            if let Some(Pin::Upper(_)) = self.direction_pin {
                self.set_gpio_upper(upper_value, upper_direction);
            }
            self.set_gpio_lower(lower_value, lower_direction | SCL);
            self
        }
        pub(super) fn start(&mut self, count: usize) -> &mut Self {
            for _ in 0..count {
                self.i2c_out(true, true);
            }
            for _ in 0..count {
                self.i2c_out(true, false);
            }
            self
        }
        pub(super) fn end(&mut self, count: usize) -> &mut Self {
            for _ in 0..count {
                self.i2c_out(true, false);
            }
            for _ in 0..count {
                self.i2c_out(true, true);
            }
            self
        }
        pub(super) fn i2c_addr(&mut self, addr: u8, is_read: bool) -> &mut Self {
            let addr = if is_read { (addr << 1) | 1 } else { addr << 1 };
            self.clock_bits_out(TCK_INIT_VALUE, IS_LSB, addr, DATA_BITS);
            self.i2c_in()
                .clock_bits_in(TCK_INIT_VALUE, IS_LSB, ACK_BITS);
            self
        }
        pub(super) fn i2c_read(&mut self, m_ack: bool) -> &mut Self {
            let m_ack = if m_ack { 0 } else { 0xff };
            self.i2c_in()
                .clock_bits_in(TCK_INIT_VALUE, IS_LSB, DATA_BITS);
            self.i2c_out(false, false)
                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, m_ack, ACK_BITS);
            self
        }
        pub(super) fn i2c_write(&mut self, value: u8) -> &mut Self {
            self.i2c_out(false, false)
                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, value, DATA_BITS);
            self.i2c_in()
                .clock_bits_in(TCK_INIT_VALUE, IS_LSB, ACK_BITS);
            self
        }
    }
}
