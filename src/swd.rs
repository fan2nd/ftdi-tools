use std::sync::{Arc, Mutex};

use self::cmd::SwdCmdBuilder;
use crate::{FtdiMpsse, Pin, PinUse, ftdaye::FtdiError};

#[derive(Debug, thiserror::Error)]
pub enum FtdiSwdError {
    #[error("Ftdi inner error")]
    FtdiInner(#[from] FtdiError),
    #[error("Swd ack wait.")]
    AckWait,
    #[error("Swd ack failed.")]
    AckFailed,
    #[error("Swd unknown ack LSB[{0:#3b}].")]
    UnknownAck(u8),
    #[error("Swd parity error.")]
    ParityError,
}

#[derive(Debug, Clone, Copy)]
pub enum SwdAddr {
    Dp(u8),
    Ap(u8),
}
impl From<SwdAddr> for u8 {
    fn from(value: SwdAddr) -> Self {
        // Timing Sequence: [Start(1), APnDP, RnW, A[2:3], Parity, Stop(0), Park(1)]
        // LSB Format: [Park(1), Stop(0), Parity, A[3:2], RnW, APnDP, Start(1)]
        const AP_MASK: u8 = 1 << 1;
        const ADDR_MASK: u8 = 0b11 << 2;
        match value {
            SwdAddr::Dp(addr) => addr << 1 & ADDR_MASK,
            SwdAddr::Ap(addr) => addr << 1 & ADDR_MASK | AP_MASK,
        }
    }
}
/// Serial Wire Debug (SWD) interface controller
/// Implements ARM Debug Interface v5 communication protocol
pub struct FtdiSwd {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Optional direction control pin for SWDIO signal (half-duplex mode)
    direction_pin: Option<Pin>,
}
impl Drop for FtdiSwd {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
        if let Some(pin) = self.direction_pin {
            lock.free_pin(pin);
        }
    }
}
impl FtdiSwd {
    // Swd ACK (3 bits)
    // LSB[2:0] - 001:成功,010:等待,100:失败
    const REPONSE_SUCCESS: u8 = 0b001;
    const REPONSE_WAIT: u8 = 0b010;
    const REPONSE_FAILED: u8 = 0b100;
    /// Initialize SWD interface
    /// Allocates and configures GPIO pins:
    ///   Pin0 (SCK)        - Output
    ///   Pin1 (DIO_OUTPUT) - Output
    ///   Pin2 (DIO_INPUT)  - Input
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSwdError> {
        {
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(Pin::Lower(0), PinUse::Swd);
            lock.alloc_pin(Pin::Lower(1), PinUse::Swd);
            lock.alloc_pin(Pin::Lower(2), PinUse::Swd);
            // clear direction and value of first 3 pins
            // pins default input and value 0
            // AD0: SCK
            // AD1: SWDIO_OUT (master out)
            // AD2: SWDIO_IN (master in)
        }
        Ok(Self {
            mtx,
            direction_pin: None,
        })
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
    /// Send SWD activation sequence
    /// Sequence: >50 ones + 0x79E7 (MSB first) + >50 ones
    pub fn enable(&self) -> Result<(), FtdiError> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin);
        cmd.swd_enable();

        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
    /// Build SWD request packet (lsb 8 bits)
    /// Timing Sequence: [Start(1), APnDP, RnW, A[2:3], Parity, Stop(0), Park(1)]
    /// LSB Format: [Park(1), Stop(0), Parity, A[3:2], RnW, APnDP, Start(1)]
    fn build_request(is_read: bool, addr: SwdAddr) -> u8 {
        const START_MASK: u8 = 1 << 0;
        const READ_MASK: u8 = 1 << 2;
        const PARITY_MASK: u8 = 1 << 5;
        const PARK_MASK: u8 = 1 << 7;
        let mut request = START_MASK | PARK_MASK; // Start(1) + Park(1) with Stop(0)
        request |= if is_read { READ_MASK } else { 0 }; // Set RnW bit (position 2)
        request |= u8::from(addr);

        // The parity check is made over the APnDP, RnW and A[2:3] bits. If, of these four bits:
        // • the number of bits set to 1 is odd, then the parity bit is set to 1
        // • the number of bits set to 1 is even, then the parity bit is set to 0.
        let parity = ((request >> 1) & 0x0F).count_ones();
        request |= if parity != 0 { PARITY_MASK } else { 0 }; // Set parity bit (position 5)

        request
    }
    /// Perform SWD read operation
    /// Performs SWD read operation from specified debug port address
    ///
    /// # Arguments
    /// * `addr` - SWD address specification (AP or DP with register offset)
    ///
    /// # Returns
    /// Result containing 32-bit read value or FtdiError if communication fails
    ///
    /// # Protocol Details
    /// Implements SWD read transaction including request, ACK check, data reception,
    /// and parity verification as defined in ARM Debug Interface Architecture Specification
    pub fn read(&self, addr: SwdAddr) -> Result<u32, FtdiSwdError> {
        let lock = self.mtx.lock().unwrap();
        let request = Self::build_request(true, addr);
        let mut response = [0u8];
        // Send request (8 bits)
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin);
        cmd.swd_send_request(request).trn().swd_read_response();
        lock.write_read(cmd.as_slice(), &mut response)?;

        // Read ACK (3 bits)
        let ack = response[0] >> 5;
        if ack != Self::REPONSE_SUCCESS {
            let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin);
            cmd.trn();
            lock.write_read(cmd.as_slice(), &mut [])?;
            match ack {
                Self::REPONSE_WAIT => return Err(FtdiSwdError::AckWait),
                Self::REPONSE_FAILED => return Err(FtdiSwdError::AckFailed),
                x => return Err(FtdiSwdError::UnknownAck(x)),
            }
        }

        // Read data (32 bits) + parity (1 bit) = 33 bits
        let mut response = [0u8; 5]; // 33 bits = 5 bytes
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin);
        cmd.swd_read_data().trn();
        lock.write_read(cmd.as_slice(), &mut response)?;

        // Parse the data (LSB first)
        let value = u32::from_le_bytes([response[0], response[1], response[2], response[3]]);
        let parity = (response[4] >> 7) & 0x01;
        let calc_parity = value.count_ones() as u8 & 0x01;

        if parity != calc_parity {
            return Err(FtdiSwdError::ParityError);
        }
        Ok(value)
    }

    pub fn write(&self, addr: SwdAddr, value: u32) -> Result<(), FtdiSwdError> {
        let lock = self.mtx.lock().unwrap();
        let request = Self::build_request(false, addr);
        let mut response = [0u8];
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin);
        cmd.swd_send_request(request)
            .trn()
            .swd_read_response()
            .trn();
        lock.write_read(cmd.as_slice(), &mut response)?;

        // Read ACK (3 bits)
        let ack = response[0] >> 5;
        if ack != Self::REPONSE_SUCCESS {
            match ack {
                Self::REPONSE_WAIT => return Err(FtdiSwdError::AckWait),
                Self::REPONSE_FAILED => return Err(FtdiSwdError::AckFailed),
                x => return Err(FtdiSwdError::UnknownAck(x)),
            }
        }
        // Send data (33 bits)
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin);
        cmd.swd_write_data(value);
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
}

mod cmd {
    const SWCLK: u8 = 1 << 0; // SWCLK bitmask
    const SWDIO: u8 = 1 << 1; // SWDIO bitmask
    const TCK_INIT_VALUE: bool = false;
    const IS_LSB: bool = true;

    use crate::{FtdiMpsse, Pin, mpsse_cmd::MpsseCmdBuilder};
    use std::{
        ops::{Deref, DerefMut},
        sync::MutexGuard,
    };
    pub(super) struct SwdCmdBuilder<'a> {
        cmd: MpsseCmdBuilder,
        lock: &'a MutexGuard<'a, FtdiMpsse>,
        direction_pin: Option<Pin>,
    }
    impl<'a> Deref for SwdCmdBuilder<'a> {
        type Target = MpsseCmdBuilder;
        fn deref(&self) -> &Self::Target {
            &self.cmd
        }
    }
    impl<'a> DerefMut for SwdCmdBuilder<'a> {
        fn deref_mut(&mut self) -> &mut Self::Target {
            &mut self.cmd
        }
    }
    impl<'a> SwdCmdBuilder<'a> {
        pub(super) fn new(lock: &'a MutexGuard<FtdiMpsse>, direction_pin: Option<Pin>) -> Self {
            SwdCmdBuilder {
                cmd: MpsseCmdBuilder::new(),
                lock,
                direction_pin,
            }
        }
        fn swd_out(&mut self) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            if let Some(pin) = self.direction_pin {
                match pin {
                    Pin::Lower(idx) => {
                        self.set_gpio_lower(
                            lower_value | (1 << idx),
                            lower_direction | SWCLK | SWDIO,
                        );
                    }
                    Pin::Upper(idx) => {
                        self.set_gpio_lower(lower_value, lower_direction | SWCLK | SWDIO);
                        self.set_gpio_upper(upper_value | (1 << idx), upper_direction);
                    }
                }
            } else {
                self.set_gpio_lower(lower_value, lower_direction | SWCLK | SWDIO);
            }
            self
        }
        fn swd_in(&mut self) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            if let Some(Pin::Upper(_)) = self.direction_pin {
                self.set_gpio_upper(upper_value, upper_direction);
            }
            self.set_gpio_lower(lower_value, lower_direction | SWCLK);
            self
        }
        pub(super) fn trn(&mut self) -> &mut Self {
            self.swd_in()
                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, 0xff, 1);
            self
        }
        pub(super) fn swd_line_reset(&mut self) -> &mut Self {
            const ONES: &[u8] = &[0xff; 7];
            const ZEOS: u8 = 0;
            self.swd_out()
                .clock_bytes_out(TCK_INIT_VALUE, IS_LSB, ONES) // >50 ones (LSB first)
                // AdiV5.2-B4.3.3
                // A line reset is achieved by holding the data signal HIGH for at least 50 clock cycles, followed by at least two idle cycles.
                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, ZEOS, 2); // >2 zeros (LSB first)
            self
        }
        pub(super) fn swd_enable(&mut self) -> &mut Self {
            const ONES: &[u8] = &[0xff; 7];
            // 0111_1001_1110_0111
            // 0x79E7, transmitted MSB first.
            // 0xE79E, transmitted least-significant-bit (LSB) first.
            const SEQUENCE: &[u8] = &0xE79E_u16.to_le_bytes();
            self.swd_out()
                .clock_bytes_out(TCK_INIT_VALUE, IS_LSB, ONES) // >50 ones
                .clock_bytes_out(TCK_INIT_VALUE, IS_LSB, SEQUENCE);
            self.swd_line_reset();
            self
        }
        pub(super) fn swd_send_request(&mut self, request: u8) -> &mut Self {
            self.swd_out()
                .clock_bytes_out(TCK_INIT_VALUE, IS_LSB, &[request]); // // Send request
            self
        }
        pub(super) fn swd_read_response(&mut self) -> &mut Self {
            const RESPONSE_BITS: usize = 3;
            self.swd_in()
                .clock_bits_in(TCK_INIT_VALUE, IS_LSB, RESPONSE_BITS);
            self
        }
        pub(super) fn swd_read_data(&mut self) -> &mut Self {
            const DATA_BYTES: usize = 4;
            const PARITY_BITS: usize = 1;
            self.swd_in()
                .clock_bytes_in(TCK_INIT_VALUE, IS_LSB, DATA_BYTES)
                .clock_bits_in(TCK_INIT_VALUE, IS_LSB, PARITY_BITS);
            self
        }
        pub(super) fn swd_write_data(&mut self, value: u32) -> &mut Self {
            const PARITY_BITS: usize = 1;
            let bytes = value.to_le_bytes();
            let parity = (value.count_ones() & 0x01) as u8;
            self.swd_out()
                .clock_bytes_out(TCK_INIT_VALUE, IS_LSB, &bytes)
                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, parity, PARITY_BITS);
            self
        }
    }
}
