//! Copy from ftdi-mpsse crate
//! Multi-protocol synchronous serial engine utilities for FTDI devices.

/// MPSSE opcodes.
///
/// Data clocking MPSSE commands are broken out into separate enums for API ergonomics:
/// [`MpsseShiftCmd`]
#[repr(u8)]
#[derive(Debug, Copy, Clone)]
enum MpsseCmd {
    /// Used by [`MpsseCmdBuilder::set_gpio_lower`].
    SetDataBitsLowbyte = 0x80,
    /// Used by [`MpsseCmdBuilder::gpio_lower`].
    GetDataBitsLowbyte = 0x81,
    /// Used by [`MpsseCmdBuilder::set_gpio_upper`].
    SetDataBitsHighbyte = 0x82,
    /// Used by [`MpsseCmdBuilder::gpio_upper`].
    GetDataBitsHighbyte = 0x83,
    /// Used by [`MpsseCmdBuilder::enable_loopback`].
    EnableLoopback = 0x84,
    /// Used by [`MpsseCmdBuilder::enable_loopback`].
    DisableLoopback = 0x85,
    /// Used by [`MpsseCmdBuilder::set_clock`].
    SetClockFrequency = 0x86,
    /// Used by [`MpsseCmdBuilder::send_immediate`].
    SendImmediate = 0x87,
    /// Used by [`MpsseCmdBuilder::_wait_on_io_high`].
    _WaitOnIOHigh = 0x88,
    /// Used by [`MpsseCmdBuilder::_wait_on_io_low`].
    _WaitOnIOLow = 0x89,
    /// Used by [`MpsseCmdBuilder::set_clock`].
    DisableClockDivideBy5 = 0x8A,
    /// Used by [`MpsseCmdBuilder::set_clock`].
    EnableClockDivideBy5 = 0x8B,
    /// Used by .
    Enable3PhaseClocking = 0x8C,
    /// Used by [`MpsseCmdBuilder::enable_3phase_data_clocking`].
    Disable3PhaseClocking = 0x8D,
    /// Used by [`MpsseCmdBuilder::enable_adaptive_clocking`].
    EnableAdaptiveClocking = 0x96,
    /// Used by [`MpsseCmdBuilder::enable_adaptive_clocking`].
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
#[bitfield_struct::bitfield(u8, order = Lsb)]
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
        is_tdi_write: bool,
        is_tdo_read: bool,
    ) -> u8 {
        assert!(
            is_tdi_write | is_tdo_read,
            "tdi_write and tdo_read can not be false tonight"
        );
        MpsseShiftCmd::new()
            .with_is_tdi_neg_write((!tck_init_value) && is_tdi_write)
            .with_is_bit_mode(is_bit_mode)
            .with_is_tdo_neg_read(tck_init_value && is_tdo_read)
            .with_is_lsb(is_lsb)
            .with_is_tdi_write(is_tdi_write)
            .with_is_tdo_read(is_tdo_read)
            .into()
    }
    fn _tms_shift(tck_init_value: bool, tdo_neg_read: bool, tdo_read: bool) -> u8 {
        MpsseShiftCmd::new()
            .with_is_tdi_neg_write(!tck_init_value)
            .with_is_tdo_neg_read(tdo_neg_read && tdo_read)
            .with_is_tdo_read(tdo_read)
            .with_is_tms_write(true)
            .into()
    }
    fn tms_shift(tdo_read: bool) -> u8 {
        // tms only be used for jtag, so tck_init_value and tdo_neg_read only can be false.
        Self::_tms_shift(false, false, tdo_read)
    }
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
const MAX_BYTES_SHIFT: usize = 65536;
const MAX_BITS_SHIFT: usize = 8;
const MAX_TMS_SHIFT: usize = 7;
#[derive(Default)]
pub(crate) struct MpsseCmdBuilder {
    cmd: Vec<u8>,
    read_len: usize,
}
impl MpsseCmdBuilder {
    /// Create a new command builder.
    pub(crate) fn new() -> MpsseCmdBuilder {
        Default::default()
    }

    /// Destruct the MPSSE command.
    pub(crate) fn destruct(mut self) -> (Vec<u8>, Vec<u8>) {
        self.send_immediate();
        (self.cmd, vec![0; self.read_len])
    }

    /// Set the MPSSE clock frequency using provided
    /// divisor value and clock divider configuration.
    /// Both parameters are device dependent.
    pub(crate) fn set_clock(&mut self, divisor: u16, clk_div_by5: Option<bool>) -> &mut Self {
        match clk_div_by5 {
            Some(true) => self.cmd.push(MpsseCmd::EnableClockDivideBy5 as u8),
            Some(false) => self.cmd.push(MpsseCmd::DisableClockDivideBy5 as u8),
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
    pub(crate) fn enable_loopback(&mut self, state: bool) -> &mut Self {
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
    pub(crate) fn enable_3phase_data_clocking(&mut self, state: bool) -> &mut Self {
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
    pub(crate) fn enable_adaptive_clocking(&mut self, state: bool) -> &mut Self {
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
    pub(crate) fn set_gpio_lower(&mut self, state: u8, direction: u8) -> &mut Self {
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
    pub(crate) fn set_gpio_upper(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsHighbyte as u8, state, direction]);
        self
    }

    /// Get the pin state state of the lower byte (0-7) GPIO pins on the MPSSE
    /// interface.
    pub(crate) fn gpio_lower(&mut self) -> &mut Self {
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
    pub(crate) fn gpio_upper(&mut self) -> &mut Self {
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
    pub(crate) fn _wait_on_io_high(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::_WaitOnIOHigh as u8);
        self
    }

    /// Make controller wait until GPIOL1 or I/O1 is low before running further commands.
    /// use crate::mpsse::{ClockBytes, MpsseCmdBuilder};
    ///
    /// // Assume a "chip ready" signal is connected to GPIOL1. This signal is pulled low
    /// // shortly after AD3 (chip select) is pulled low. Data will not be clocked out until
    /// // the chip is ready.
    pub(crate) fn _wait_on_io_low(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::_WaitOnIOLow as u8);
        self
    }

    /// Clock data out.
    ///
    /// This will clock out bytes on TDI/DO.
    /// No data is clocked into the device on TDO/DI.
    ///
    /// This will panic for data lengths greater than `u16::MAX + 1`.
    pub(crate) fn shift_bytes_out(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        for slice in data.chunks(MAX_BYTES_SHIFT) {
            self.shift_bytes_out_limited(tck_init_value, is_lsb, slice);
        }
        self
    }
    fn shift_bytes_out_limited(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        let mut len = data.len();
        if len == 0 {
            return self;
        }
        assert!(
            len <= MAX_BYTES_SHIFT,
            "data length should be less than {MAX_BYTES_SHIFT}"
        );
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, true, false),
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
    pub(crate) fn shift_bytes_in(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        mut len: usize,
    ) -> &mut Self {
        while len >= MAX_BYTES_SHIFT {
            self.shift_bytes_in_limited(tck_init_value, is_lsb, MAX_BYTES_SHIFT);
            len -= MAX_BYTES_SHIFT;
        }
        self.shift_bytes_in_limited(tck_init_value, is_lsb, len);
        self
    }
    fn shift_bytes_in_limited(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        mut len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(
            len <= MAX_BYTES_SHIFT,
            "data length should be less than {MAX_BYTES_SHIFT}"
        );
        self.read_len += len;
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, false, true),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self
    }

    /// Clock data in and out simultaneously.
    ///
    /// This will panic for data lengths greater than `u16::MAX + 1`.
    pub(crate) fn shift_bytes(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        for slice in data.chunks(MAX_BYTES_SHIFT) {
            self.shift_bytes_limited(tck_init_value, is_lsb, slice);
        }
        self
    }
    fn shift_bytes_limited(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        let mut len = data.len();
        if len == 0 {
            return self;
        }
        assert!(
            len <= MAX_BYTES_SHIFT,
            "data length should be less than {MAX_BYTES_SHIFT}"
        );
        self.read_len += len;
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, true, true),
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
    pub(crate) fn shift_bits_out(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: u8,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be less than {MAX_BITS_SHIFT}");
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, true, false),
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
    pub(crate) fn shift_bits_in(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be less than {MAX_BITS_SHIFT}");
        self.read_len += 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, false, true),
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
    pub(crate) fn shift_bits(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: u8,
        len: usize,
    ) -> &mut Self {
        // Normally len will only be 1.
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be less than {MAX_BITS_SHIFT}");

        self.read_len += 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, true, true),
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
    pub(crate) fn clock_tms_out(&mut self, tdi: bool, data: u8, len: usize) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 7, "data length should be less than {MAX_TMS_SHIFT}");
        let data = if tdi { data | 0x80 } else { data };
        self.cmd
            .extend_from_slice(&[MpsseShiftCmd::tms_shift(false), (len - 1) as u8, data]);
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
    pub(crate) fn clock_tms(&mut self, tdi: bool, data: u8, len: usize) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 7, "data length should be less than {MAX_TMS_SHIFT}");
        self.read_len += 1;
        let data = if tdi { data | 0x80 } else { data };
        self.cmd
            .extend_from_slice(&[MpsseShiftCmd::tms_shift(true), (len - 1) as u8, data]);
        self
    }
}
#[cfg(test)]
mod test {
    use super::MpsseShiftCmd;
    #[test]
    fn mpsse_shift_cmd_write_box_test() {
        // AN108 3.3
        assert_eq!(
            0x10u8,
            MpsseShiftCmd::shift(true, false, false, true, false)
        );
        assert_eq!(
            0x11u8,
            MpsseShiftCmd::shift(false, false, false, true, false)
        );
        assert_eq!(0x12u8, MpsseShiftCmd::shift(true, true, false, true, false));
        assert_eq!(
            0x13u8,
            MpsseShiftCmd::shift(false, true, false, true, false)
        );

        assert_eq!(
            0x20u8,
            MpsseShiftCmd::shift(false, false, false, false, true)
        );
        assert_eq!(
            0x24u8,
            MpsseShiftCmd::shift(true, false, false, false, true)
        );
        assert_eq!(
            0x22u8,
            MpsseShiftCmd::shift(false, true, false, false, true)
        );
        assert_eq!(0x26u8, MpsseShiftCmd::shift(true, true, false, false, true));

        assert_eq!(
            0x31u8,
            MpsseShiftCmd::shift(false, false, false, true, true)
        );
        assert_eq!(0x34u8, MpsseShiftCmd::shift(true, false, false, true, true));
        assert_eq!(0x33u8, MpsseShiftCmd::shift(false, true, false, true, true));
        assert_eq!(0x36u8, MpsseShiftCmd::shift(true, true, false, true, true));

        // AN108-3.4
        assert_eq!(0x18u8, MpsseShiftCmd::shift(true, false, true, true, false));
        assert_eq!(
            0x19u8,
            MpsseShiftCmd::shift(false, false, true, true, false)
        );
        assert_eq!(0x1au8, MpsseShiftCmd::shift(true, true, true, true, false));
        assert_eq!(0x1bu8, MpsseShiftCmd::shift(false, true, true, true, false));

        assert_eq!(
            0x28u8,
            MpsseShiftCmd::shift(false, false, true, false, true)
        );
        assert_eq!(0x2cu8, MpsseShiftCmd::shift(true, false, true, false, true));
        assert_eq!(0x2au8, MpsseShiftCmd::shift(false, true, true, false, true));
        assert_eq!(0x2eu8, MpsseShiftCmd::shift(true, true, true, false, true));

        assert_eq!(0x39u8, MpsseShiftCmd::shift(false, false, true, true, true));
        assert_eq!(0x3cu8, MpsseShiftCmd::shift(true, false, true, true, true));
        assert_eq!(0x3bu8, MpsseShiftCmd::shift(false, true, true, true, true));
        assert_eq!(0x3eu8, MpsseShiftCmd::shift(true, true, true, true, true));

        // AN108-3.5
        // NOTE: The table in 3.5 is not correct.
        assert_eq!(0x4au8, MpsseShiftCmd::_tms_shift(true, false, false)); // Not used.
        assert_eq!(0x4au8, MpsseShiftCmd::_tms_shift(true, true, false)); // Not used.
        assert_eq!(0x4bu8, MpsseShiftCmd::_tms_shift(false, false, false));
        assert_eq!(0x4bu8, MpsseShiftCmd::tms_shift(false));
        assert_eq!(0x4bu8, MpsseShiftCmd::_tms_shift(false, true, false)); // Not used.
        assert_eq!(0x6au8, MpsseShiftCmd::_tms_shift(true, false, true)); // Not used.
        assert_eq!(0x6bu8, MpsseShiftCmd::_tms_shift(false, false, true));
        assert_eq!(0x6bu8, MpsseShiftCmd::tms_shift(true));
        assert_eq!(0x6eu8, MpsseShiftCmd::_tms_shift(true, true, true)); // Not used.
        assert_eq!(0x6fu8, MpsseShiftCmd::_tms_shift(false, true, true)); // Not used.
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
                MpsseShiftCmd::_tms_shift(permutation[0], permutation[1], permutation[2])
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
}
