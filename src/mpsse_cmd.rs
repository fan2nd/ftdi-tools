//! Copy from ftdi-mpsse crate
//! Multi-protocol synchronous serial engine utilities for FTDI devices.

/// MPSSE opcodes.
///
/// Exported for use by [`mpsse`] macro. May also be used for manual command array construction.
///
/// Data clocking MPSSE commands are broken out into separate enums for API ergonomics:
/// * [`MpsseShiftCmd`]
#[repr(u8)]
#[derive(Debug, Copy, Clone)]
enum MpsseCmd {
    /// Used by [`set_gpio_lower`][`MpsseCmdBuilder::set_gpio_lower`].
    SetDataBitsLowbyte = 0x80,
    /// Used by [`gpio_lower`][`MpsseCmdBuilder::gpio_lower`].
    GetDataBitsLowbyte = 0x81,
    /// Used by [`set_gpio_upper`][`MpsseCmdBuilder::set_gpio_upper`].
    SetDataBitsHighbyte = 0x82,
    /// Used by [`gpio_upper`][`MpsseCmdBuilder::gpio_upper`].
    GetDataBitsHighbyte = 0x83,
    /// Used by [`enable_loopback`][`MpsseCmdBuilder::enable_loopback`].
    EnableLoopback = 0x84,
    /// Used by [`enable_loopback`][`MpsseCmdBuilder::enable_loopback`].
    DisableLoopback = 0x85,
    /// Used by [`set_clock`][`MpsseCmdBuilder::set_clock`].
    SetClockFrequency = 0x86,
    /// Used by [`send_immediate`][`MpsseCmdBuilder::send_immediate`].
    SendImmediate = 0x87,
    /// Used by [`_wait_on_io_high`][`MpsseCmdBuilder::_wait_on_io_high`].
    _WaitOnIOHigh = 0x88,
    /// Used by [`_wait_on_io_low`][`MpsseCmdBuilder::_wait_on_io_low`].
    _WaitOnIOLow = 0x89,
    /// Used by [`set_clock`][`MpsseCmdBuilder::set_clock`].
    DisableClockDivide = 0x8A,
    /// Used by [`set_clock`][`MpsseCmdBuilder::set_clock`].
    EnableClockDivide = 0x8B,
    /// Used by [`enable_3phase_data_clocking`][`MpsseCmdBuilder::enable_3phase_data_clocking`].
    Enable3PhaseClocking = 0x8C,
    /// Used by [`enable_3phase_data_clocking`][`MpsseCmdBuilder::enable_3phase_data_clocking`].
    Disable3PhaseClocking = 0x8D,
    /// Used by [`enable_adaptive_clocking`][`MpsseCmdBuilder::enable_adaptive_clocking`].
    EnableAdaptiveClocking = 0x96,
    /// Used by [`enable_adaptive_clocking`][`MpsseCmdBuilder::enable_adaptive_clocking`].
    DisableAdaptiveClocking = 0x97,
    // This command is only available to FT232
    _EnableDriveOnlyZero = 0x9E,
}
/// Command for data shift of the FTDI device.
///
/// All the command it constructed are vaild. According to two following test function.
///
/// When tms_write is false:
///
/// TDI(AD1) can only can output on second edge.
///
/// TDO(AD2) can only can sample on first edge.
///
/// When tms_write is true:
///
/// TMS(AD3) can only can output on second edge.
#[bitfield_struct::bitfield(u8)]
struct MpsseShiftCmd {
    is_tdi_neg_write: bool,
    #[bits(default = true)] // when tms enable, this const true
    is_bit_mode: bool,
    is_tdo_neg_read: bool,
    #[bits(default = true)] // when tms enable, this const true
    is_lsb: bool,
    is_tdi_write: bool,
    is_tdo_read: bool,
    #[bits(default = false)] // tms is used less frequency
    is_tms_write: bool,
    #[bits(default = false)]
    _const_0: bool,
}
impl MpsseShiftCmd {
    fn shift(
        tck_init_value: bool,
        is_bit_mode: bool,
        is_lsb: bool,
        tdi_write: bool,
        tdo_read: bool,
    ) -> Self {
        assert!(
            tdi_write | tdo_read,
            "tdi_write and tdo_read can not be false tonight"
        );
        MpsseShiftCmd::new()
            .with_is_tdi_neg_write((!tck_init_value) && tdi_write)
            .with_is_bit_mode(is_bit_mode)
            .with_is_tdo_neg_read(tck_init_value && tdo_read)
            .with_is_lsb(is_lsb)
            .with_is_tdi_write(tdi_write)
            .with_is_tdo_read(tdo_read)
    }
    fn tms_shift(tck_init_value: bool, tdo_neg_read: bool, tdo_read: bool) -> Self {
        MpsseShiftCmd::new()
            .with_is_tdi_neg_write(!tck_init_value)
            .with_is_tdo_neg_read(tdo_neg_read && tdo_read)
            .with_is_tdo_read(tdo_read)
            .with_is_tms_write(true)
    }
}
#[test]
fn mpsse_shift_cmd_write_box_test() {
    // AN108 3.3
    assert_eq!(
        0x10 as u8,
        MpsseShiftCmd::shift(true, false, false, true, false).into()
    );
    assert_eq!(
        0x11 as u8,
        MpsseShiftCmd::shift(false, false, false, true, false).into()
    );
    assert_eq!(
        0x12 as u8,
        MpsseShiftCmd::shift(true, true, false, true, false).into()
    );
    assert_eq!(
        0x13 as u8,
        MpsseShiftCmd::shift(false, true, false, true, false).into()
    );

    assert_eq!(
        0x20 as u8,
        MpsseShiftCmd::shift(false, false, false, false, true).into()
    );
    assert_eq!(
        0x24 as u8,
        MpsseShiftCmd::shift(true, false, false, false, true).into()
    );
    assert_eq!(
        0x22 as u8,
        MpsseShiftCmd::shift(false, true, false, false, true).into()
    );
    assert_eq!(
        0x26 as u8,
        MpsseShiftCmd::shift(true, true, false, false, true).into()
    );

    assert_eq!(
        0x31 as u8,
        MpsseShiftCmd::shift(false, false, false, true, true).into()
    );
    assert_eq!(
        0x34 as u8,
        MpsseShiftCmd::shift(true, false, false, true, true).into()
    );
    assert_eq!(
        0x33 as u8,
        MpsseShiftCmd::shift(false, true, false, true, true).into()
    );
    assert_eq!(
        0x36 as u8,
        MpsseShiftCmd::shift(true, true, false, true, true).into()
    );

    // AN108-3.4
    assert_eq!(
        0x18 as u8,
        MpsseShiftCmd::shift(true, false, true, true, false).into()
    );
    assert_eq!(
        0x19 as u8,
        MpsseShiftCmd::shift(false, false, true, true, false).into()
    );
    assert_eq!(
        0x1a as u8,
        MpsseShiftCmd::shift(true, true, true, true, false).into()
    );
    assert_eq!(
        0x1b as u8,
        MpsseShiftCmd::shift(false, true, true, true, false).into()
    );

    assert_eq!(
        0x28 as u8,
        MpsseShiftCmd::shift(false, false, true, false, true).into()
    );
    assert_eq!(
        0x2c as u8,
        MpsseShiftCmd::shift(true, false, true, false, true).into()
    );
    assert_eq!(
        0x2a as u8,
        MpsseShiftCmd::shift(false, true, true, false, true).into()
    );
    assert_eq!(
        0x2e as u8,
        MpsseShiftCmd::shift(true, true, true, false, true).into()
    );

    assert_eq!(
        0x39 as u8,
        MpsseShiftCmd::shift(false, false, true, true, true).into()
    );
    assert_eq!(
        0x3c as u8,
        MpsseShiftCmd::shift(true, false, true, true, true).into()
    );
    assert_eq!(
        0x3b as u8,
        MpsseShiftCmd::shift(false, true, true, true, true).into()
    );
    assert_eq!(
        0x3e as u8,
        MpsseShiftCmd::shift(true, true, true, true, true).into()
    );

    // AN108-3.5
    // Note: The table in 3.5 is not correct.
    assert_eq!(
        0x4a as u8,
        MpsseShiftCmd::tms_shift(true, false, false).into()
    );
    assert_eq!(
        0x4a as u8,
        MpsseShiftCmd::tms_shift(true, true, false).into()
    );
    assert_eq!(
        0x4b as u8,
        MpsseShiftCmd::tms_shift(false, false, false).into()
    );
    assert_eq!(
        0x4b as u8,
        MpsseShiftCmd::tms_shift(false, true, false).into()
    );
    assert_eq!(
        0x6a as u8,
        MpsseShiftCmd::tms_shift(true, false, true).into()
    );
    assert_eq!(
        0x6b as u8,
        MpsseShiftCmd::tms_shift(false, false, true).into()
    );
    assert_eq!(
        0x6e as u8,
        MpsseShiftCmd::tms_shift(true, true, true).into()
    );
    assert_eq!(
        0x6f as u8,
        MpsseShiftCmd::tms_shift(false, true, true).into()
    );
}
#[test]
fn mpsse_shift_cmd_black_box_test() {
    use std::panic;
    let mut cmd_set = Vec::new();
    let permutations: Vec<_> = (0..32)
        .map(|i| {
            [
                i & (1 << 0) != 0, // 最低位 -> 索引0
                i & (1 << 1) != 0,
                i & (1 << 2) != 0,
                i & (1 << 3) != 0,
                i & (1 << 4) != 0, // 最高位 -> 索引4
            ]
        })
        .collect();
    for permutation in permutations.iter() {
        let result: Result<u8, _> = panic::catch_unwind(|| {
            MpsseShiftCmd::shift(
                permutation[0],
                permutation[1],
                permutation[2],
                permutation[3],
                permutation[4],
            )
            .into()
        });
        if let Ok(x) = result {
            cmd_set.push(x);
        }
    }
    for permutation in permutations[0..8].iter() {
        let result: Result<u8, _> = panic::catch_unwind(|| {
            MpsseShiftCmd::tms_shift(permutation[0], permutation[1], permutation[2]).into()
        });
        if let Ok(x) = result {
            cmd_set.push(x);
        }
    }
    cmd_set.sort();
    cmd_set.dedup();
    assert_eq!(
        cmd_set,
        [
            0x10, 0x11, 0x12, 0x13, 0x18, 0x19, 0x1a, 0x1b, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2a,
            0x2c, 0x2e, 0x31, 0x33, 0x34, 0x36, 0x39, 0x3b, 0x3c, 0x3e, 0x4a, 0x4b, 0x6a, 0x6b,
            0x6e, 0x6f
        ]
    )
}

/// FTDI Multi-Protocol Synchronous Serial Engine (MPSSE) command builder.
///
/// For details about the MPSSE read the [FTDI MPSSE Basics].
///
/// This structure is a `Vec<u8>` that the methods push bytewise commands onto.
/// These commands can then be written to the device with the appropriate
/// implementations of [`send`] and [`xfer`] methods.
///
/// This is useful for creating commands that need to do multiple operations
/// quickly, since individual write calls can be expensive. For example,
/// this can be used to set a GPIO low and clock data out for SPI operations.
///
/// If dynamic command layout is not required, the [`mpsse`] macro can build
/// command `[u8; N]` arrays at compile-time.
///
/// [FTDI MPSSE Basics]: https://www.ftdichip.com/Support/Documents/AppNotes/AN_135_MPSSE_Basics.pdf
/// [`send`]: MpsseCmdExecutor::send
/// [`xfer`]: MpsseCmdExecutor::xfer
#[derive(Default)]
pub struct MpsseCmdBuilder {
    cmd: Vec<u8>,
    read_len: usize,
}
impl MpsseCmdBuilder {
    /// Create a new command builder.
    pub fn new() -> MpsseCmdBuilder {
        Default::default()
    }

    /// Get the MPSSE command as a slice.
    pub fn as_slice(&mut self) -> &[u8] {
        self.send_immediate().cmd.as_slice()
    }

    /// Get the response length of current MPSSE command.
    pub fn read_len(&self) -> usize {
        self.read_len
    }

    /// Set the MPSSE clock frequency using provided
    /// divisor value and clock divider configuration.
    /// Both parameters are device dependent.
    pub fn set_clock(&mut self, divisor: u16, clkdiv: Option<bool>) -> &mut Self {
        match clkdiv {
            Some(true) => self.cmd.push(MpsseCmd::EnableClockDivide as u8),
            Some(false) => self.cmd.push(MpsseCmd::DisableClockDivide as u8),
            None => {}
        };
        self.cmd.extend_from_slice(&[
            MpsseCmd::SetClockFrequency as u8,
            (divisor & 0xFF) as u8,
            ((divisor >> 8) & 0xFF) as u8,
        ]);
        self
    }

    /// MPSSE loopback state.
    pub fn enable_loopback(&mut self, state: bool) -> &mut Self {
        if state {
            self.cmd.push(MpsseCmd::EnableLoopback as u8);
        } else {
            self.cmd.push(MpsseCmd::DisableLoopback as u8);
        }
        self
    }

    /// Enable 3 phase data clocking.
    ///
    /// This is only available on FTx232H devices.
    ///
    /// This will give a 3 stage data shift for the purposes of supporting
    /// interfaces such as I2C which need the data to be valid on both edges of
    /// the clock.
    ///
    /// It will appears as:
    ///
    /// 1. Data setup for 1/2 clock period
    /// 2. Pulse clock for 1/2 clock period
    /// 3. Data hold for 1/2 clock period
    ///
    /// Disable 3 phase data clocking.
    ///
    /// This will give a 2 stage data shift which is the default state.
    ///
    /// It will appears as:
    ///
    /// 1. Data setup for 1/2 clock period
    /// 2. Pulse clock for 1/2 clock period
    pub fn enable_3phase_data_clocking(&mut self, state: bool) -> &mut Self {
        if state {
            self.cmd.push(MpsseCmd::Enable3PhaseClocking as u8);
        } else {
            self.cmd.push(MpsseCmd::Disable3PhaseClocking as u8);
        }
        self
    }

    /// Enable adaptive clocking.
    ///
    /// This is only available on FTx232H devices.
    pub fn enable_adaptive_clocking(&mut self, state: bool) -> &mut Self {
        if state {
            self.cmd.push(MpsseCmd::EnableAdaptiveClocking as u8);
        } else {
            self.cmd.push(MpsseCmd::DisableAdaptiveClocking as u8);
        }
        self
    }

    /// Set the pin direction and state of the lower byte (0-7) GPIO pins on the
    /// MPSSE interface.
    ///
    /// The pins that this controls depends on the device.
    ///
    /// * On the FT232H this will control the AD0-AD7 pins.
    ///
    /// # Arguments
    ///
    /// * `state` - GPIO state mask, `0` is low (or input pin), `1` is high.
    /// * `direction` - GPIO direction mask, `0` is input, `1` is output.
    pub fn set_gpio_lower(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsLowbyte as u8, state, direction]);
        self
    }

    /// Set the pin direction and state of the upper byte (8-15) GPIO pins on
    /// the MPSSE interface.
    ///
    /// The pins that this controls depends on the device.
    /// This method may do nothing for some devices, such as the FT4232H that
    /// only have 8 pins per port.
    ///
    /// # Arguments
    ///
    /// * `state` - GPIO state mask, `0` is low (or input pin), `1` is high.
    /// * `direction` - GPIO direction mask, `0` is input, `1` is output.
    ///
    /// # FT232H Corner Case
    ///
    /// On the FT232H only CBUS5, CBUS6, CBUS8, and CBUS9 can be controlled.
    /// These pins confusingly map to the first four bits in the direction and
    /// state masks.
    pub fn set_gpio_upper(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsHighbyte as u8, state, direction]);
        self
    }

    /// Get the pin state state of the lower byte (0-7) GPIO pins on the MPSSE
    /// interface.
    pub fn gpio_lower(&mut self) -> &mut Self {
        self.read_len += 1;
        self.cmd.push(MpsseCmd::GetDataBitsLowbyte as u8);
        self
    }

    /// Get the pin state state of the upper byte (8-15) GPIO pins on the MPSSE
    /// interface.
    ///
    /// See [`set_gpio_upper`] for additional information about physical pin
    /// mappings.
    ///
    /// [`set_gpio_upper`]: MpsseCmdBuilder::set_gpio_upper
    pub fn gpio_upper(&mut self) -> &mut Self {
        self.read_len += 1;
        self.cmd.push(MpsseCmd::GetDataBitsHighbyte as u8);
        self
    }

    /// Send the preceding commands immediately.
    fn send_immediate(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::SendImmediate as u8);
        self
    }

    /// Make controller wait until GPIOL1 or I/O1 is high before running further commands.
    /// use crate::mpsse::{ClockBytes, MpsseCmdBuilder};
    ///
    /// // Assume a "chip ready" signal is connected to GPIOL1. This signal is pulled high
    /// // shortly after AD3 (chip select) is pulled low. Data will not be clocked out until
    /// // the chip is ready.
    pub fn _wait_on_io_high(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::_WaitOnIOHigh as u8);
        self
    }

    /// Make controller wait until GPIOL1 or I/O1 is low before running further commands.
    /// use crate::mpsse::{ClockBytes, MpsseCmdBuilder};
    ///
    /// // Assume a "chip ready" signal is connected to GPIOL1. This signal is pulled low
    /// // shortly after AD3 (chip select) is pulled low. Data will not be clocked out until
    /// // the chip is ready.
    pub fn _wait_on_io_low(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::_WaitOnIOLow as u8);
        self
    }

    /// Clock data out.
    ///
    /// This will clock out bytes on TDI/DO.
    /// No data is clocked into the device on TDO/DI.
    ///
    /// This will panic for data lengths greater than `u16::MAX + 1`.
    pub fn clock_bytes_out(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        let mut len = data.len();
        if len == 0 {
            return self;
        }
        assert!(len <= 65536, "data length should be in 1..=65536");
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, true, false).into(),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self.cmd.extend_from_slice(data);
        self
    }

    /// Clock data in.
    ///
    /// This will clock in bytes on TDO/DI.
    /// No data is clocked out of the device on TDI/DO.
    ///
    /// # Arguments
    ///
    /// * `mode` - Data clocking mode.
    /// * `len` - Number of bytes to clock in.
    ///   This will panic for values greater than `u16::MAX + 1`.
    pub fn clock_bytes_in(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        mut len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 65536, "data length should be in 1..=65536");
        self.read_len += len;
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, false, true).into(),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self
    }

    /// Clock data in and out simultaneously.
    ///
    /// This will panic for data lengths greater than `u16::MAX + 1`.
    pub fn clock_bytes(&mut self, tck_init_value: bool, is_lsb: bool, data: &[u8]) -> &mut Self {
        let mut len = data.len();
        if len == 0 {
            return self;
        }
        assert!(len <= 65536, "data length should be in 1..=65536");
        self.read_len += len;
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, true, true).into(),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self.cmd.extend_from_slice(data);
        self
    }

    /// Clock data bits out.
    ///
    /// # Arguments
    ///
    /// * `mode` - Bit clocking mode.
    /// * `data` - Data bits.
    /// * `len` - Number of bits to clock out.
    ///   This will panic for values greater than 8.
    pub fn clock_bits_out(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: u8,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be in 1..=8");
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, true, false).into(),
            (len - 1) as u8,
            data,
        ]);
        self
    }

    /// Clock data bits in.
    ///
    /// # Arguments
    ///
    /// * `mode` - Bit clocking mode.
    /// * `len` - Number of bits to clock in.
    ///   This will panic for values greater than 8.
    pub fn clock_bits_in(&mut self, tck_init_value: bool, is_lsb: bool, len: usize) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be in 1..=8");
        self.read_len += 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, false, true).into(),
            (len - 1) as u8,
        ]);
        self
    }

    /// Clock data bits in and out simultaneously.
    ///
    /// # Arguments
    ///
    /// * `mode` - Bit clocking mode.
    /// * `len` - Number of bits to clock in.
    ///   This will panic for values greater than 8.
    pub fn clock_bits(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: u8,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be in 1..=8");
        self.read_len += 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, true, true).into(),
            (len - 1) as u8,
            data,
        ]);
        self
    }

    /// Clock TMS bits out.
    ///
    /// # Arguments
    ///
    /// * `mode` - TMS clocking mode.
    /// * `data` - TMS bits.
    /// * `tdi` - Value to place on TDI while clocking.
    /// * `len` - Number of bits to clock out.
    ///   This will panic for values greater than 7.
    pub fn clock_tms_out(
        &mut self,
        tck_init_value: bool,
        tdo_neg_read: bool,
        data: u8,
        tdi: bool,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 7, "data length should be in 1..=7");
        let data = if tdi { data | 0x80 } else { data };
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::tms_shift(tck_init_value, tdo_neg_read, false).into(),
            (len - 1) as u8,
            data,
        ]);
        self
    }

    /// Clock TMS bits out while clocking TDO bits in.
    ///
    /// # Arguments
    ///
    /// * `mode` - TMS clocking mode.
    /// * `data` - TMS bits.
    /// * `tdi` - Value to place on TDI while clocking.
    /// * `len` - Number of bits to clock out.
    ///   This will panic for values greater than 7.
    pub fn clock_tms(
        &mut self,
        tck_init_value: bool,
        tdo_neg_read: bool,
        data: u8,
        tdi: bool,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 7, "data length should be in 1..=7");
        self.read_len += 1;
        let data = if tdi { data | 0x80 } else { data };
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::tms_shift(tck_init_value, tdo_neg_read, false).into(),
            (len - 1) as u8,
            data,
        ]);
        self
    }
}
