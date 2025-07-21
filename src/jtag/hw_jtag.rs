use crate::ftdaye::FtdiError;
use crate::mpsse_cmd::MpsseCmdBuilder;
use crate::{FtdiMpsse, FtdiOutputPin, Pin, PinUse};
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};

const TCK_MASK: u8 = 1 << 0;
const TDI_MASK: u8 = 1 << 1;
#[allow(unused)]
const TDO_MASK: u8 = 1 << 2;
const TMS_MASK: u8 = 1 << 3;
// TCK(AD0) must be init with value 0.
// TDI(AD1) can only can output on second edge.
// TDO(AD2) can only can sample on first edge.
// according to AN108-2.2.
// https://ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
const TCK_INIT_VALUE: bool = false;
const IS_LSB: bool = true;
const TDO_NEG_READ: bool = false;

pub struct JtagCmdBuilder(MpsseCmdBuilder);
impl JtagCmdBuilder {
    fn new() -> Self {
        JtagCmdBuilder(MpsseCmdBuilder::new())
    }
    fn jtag_any2idle(&mut self) -> &mut Self {
        self.0.clock_tms_out(TCK_INIT_VALUE, 0b0001_1111, true, 6);
        self
    }
    fn jtag_idle_cycle(&mut self) -> &mut Self {
        self.0.clock_tms_out(TCK_INIT_VALUE, 0, true, 7);
        self
    }
    fn jtag_idle2ir(&mut self) -> &mut Self {
        self.0.clock_tms_out(TCK_INIT_VALUE, 0b0000_0011, true, 4);
        self
    }
    fn jtag_ir_exit2dr(&mut self) -> &mut Self {
        self.0.clock_tms_out(TCK_INIT_VALUE, 0b0000_0011, true, 4);
        self
    }
    fn jtag_idle2dr(&mut self) -> &mut Self {
        self.0.clock_tms_out(TCK_INIT_VALUE, 0b0000_0001, true, 3);
        self
    }
    fn jtag_dr_exit2idle(&mut self) -> &mut Self {
        self.0.clock_tms_out(TCK_INIT_VALUE, 0b0000_0001, true, 2);
        self
    }
    fn jtag_shift(&mut self, data: &[u8], bits_count: usize) -> &mut Self {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        let last_bit = data[bytes_count] >> remain_bits == 1;
        self.clock_bytes(TCK_INIT_VALUE, IS_LSB, &data[0..bytes_count])
            .clock_bits(TCK_INIT_VALUE, IS_LSB, data[bytes_count], remain_bits)
            .clock_tms(TCK_INIT_VALUE, TDO_NEG_READ, 0b0000_0001, last_bit, 1);
        self
    }
    fn jtag_shift_write(&mut self, data: &[u8], bits_count: usize) -> &mut Self {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        let last_bit = data[bytes_count] >> remain_bits == 1;
        self.clock_bytes_out(TCK_INIT_VALUE, IS_LSB, &data[0..bytes_count])
            .clock_bits_out(TCK_INIT_VALUE, IS_LSB, data[bytes_count], remain_bits)
            .clock_tms_out(TCK_INIT_VALUE, 0b0000_0001, last_bit, 1);
        self
    }
    fn jtag_shift_read(&mut self, bits_count: usize) -> &mut Self {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        let last_bit = Default::default(); // 
        self.clock_bytes_in(TCK_INIT_VALUE, IS_LSB, bytes_count)
            .clock_bits_in(TCK_INIT_VALUE, IS_LSB, remain_bits)
            .clock_tms(TCK_INIT_VALUE, TDO_NEG_READ, 0b0000_0001, last_bit, 1);
        self
    }
    fn jtag_parse_single_shift(response: &mut [u8], bits_count: usize) -> usize {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        if remain_bits == 0 {
            response[bytes_count] >>= 7
        } else {
            response[bytes_count] >>= 8 - remain_bits;
            response[bytes_count] |= (response[bytes_count + 1] & 0b1000_0000) >> (7 - remain_bits);
        }
        bytes_count + 1
    }
}
impl Deref for JtagCmdBuilder {
    type Target = MpsseCmdBuilder;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for JtagCmdBuilder {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
/// JTAG (Joint Test Action Group) interface controller
/// Implements JTAG state machine management and data transfer operations
pub struct FtdiJtag {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Tracks if the JTAG state machine is in idle state
    is_idle: bool,
    /// Whether adaptive clocking (RTCK) is enabled
    adaptive_clocking: bool,
    /// Optional custom pin assignments for JTAG signals
    direction: Option<[FtdiOutputPin; 4]>,
}
impl Drop for FtdiJtag {
    fn drop(&mut self) {
        self.adaptive_clock(false).unwrap();
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
        lock.free_pin(Pin::Lower(3));
    }
}
impl FtdiJtag {
    /// Creates a new JTAG interface instance
    ///
    /// # Arguments
    /// * `mtx` - Thread-safe handle to FTDI MPSSE controller
    ///
    /// # Returns
    /// Result containing Jtag instance or FtdiError
    ///
    /// # Pin Allocation
    /// Default pin assignments on lower GPIO bank:
    /// - TCK: Lower(0) - Test Clock
    /// - TDI: Lower(1) - Test Data In
    /// - TDO: Lower(2) - Test Data Out
    /// - TMS: Lower(3) - Test Mode Select
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiError> {
        {
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(Pin::Lower(0), PinUse::Jtag); // TCK
            lock.alloc_pin(Pin::Lower(1), PinUse::Jtag); // TDI
            lock.alloc_pin(Pin::Lower(2), PinUse::Jtag); // TDO (input)
            lock.alloc_pin(Pin::Lower(3), PinUse::Jtag); // TMS
            // Set TCK, TDI, TMS as output pins (0x0b = 00001011)
            lock.lower.direction |= TCK_MASK | TDI_MASK | TMS_MASK;
            // TCK must initialize to low (AN108-2.2)
            // TDI outputs on second clock edge
            // TDO samples on first clock edge
            // Reference: https://ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(Self {
            mtx,
            is_idle: false,
            adaptive_clocking: false,
            direction: Default::default(),
        })
    }
    /// Enables/disables adaptive clocking (RTCK)
    ///
    /// # Arguments
    /// * `state` - true to enable adaptive clocking, false to disable
    ///
    /// # Notes
    /// Requires JTAG target to support RTCK feedback
    pub fn adaptive_clock(&mut self, state: bool) -> Result<(), FtdiError> {
        if self.adaptive_clocking == state {
            return Ok(());
        }
        let mut lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        if state {
            log::info!("Use {:?} as RTCK.", Pin::Lower(7));
            lock.alloc_pin(Pin::Lower(7), PinUse::Jtag);
        } else {
            log::info!("Free {:?}.", Pin::Lower(7));
            lock.free_pin(Pin::Lower(7));
        }
        cmd.enable_adaptive_clocking(state);
        lock.write_read(cmd.as_slice(), &mut [])?;
        self.adaptive_clocking = state;
        Ok(())
    }
    /// Configures custom JTAG pin assignments
    ///
    /// # Arguments
    /// * `tck` - Test Clock pin
    /// * `tdi` - Test Data Input pin
    /// * `tdo` - Test Data Output pin
    /// * `tms` - Test Mode Select pin
    ///
    /// # Important
    /// Must be called after initialization, original default pins will be overridden
    pub fn with_direction(
        &mut self,
        tck: Pin,
        tdi: Pin,
        tdo: Pin,
        tms: Pin,
    ) -> Result<(), FtdiError> {
        let tck = FtdiOutputPin::new(self.mtx.clone(), tck)?;
        let tdi = FtdiOutputPin::new(self.mtx.clone(), tdi)?;
        let tdo = FtdiOutputPin::new(self.mtx.clone(), tdo)?;
        let tms = FtdiOutputPin::new(self.mtx.clone(), tms)?;
        tck.set(true)?;
        tdi.set(true)?;
        tdo.set(false)?;
        tms.set(true)?;
        self.direction = Some([tck, tdi, tdo, tms]);
        Ok(())
    }
    pub fn goto_idle(&mut self) -> Result<(), FtdiError> {
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle();
        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut [])?;
        self.is_idle = true;
        Ok(())
    }
    pub fn scan_with(&mut self, tdi: bool) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle().jtag_idle2dr();
        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut [])?;
        let tdi = if tdi { vec![0xff; 4] } else { vec![0; 4] };
        // 移入0并读取TDO，持续直到检测到连续32个0
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_zeros = 0;

        'outer: loop {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.clock_bytes(TCK_INIT_VALUE, IS_LSB, &tdi);
            let mut response = [0u8; 4];
            lock.write_read(cmd.as_slice(), &mut response)?;
            let tdos: Vec<_> = response
                .iter()
                .flat_map(|&byte| (0..8).map(move |i| (byte >> i) & 1 == 1))
                .collect();
            for tdo_val in tdos {
                // bypass
                if bit_count == 0 && !tdo_val {
                    idcodes.push(None);
                    consecutive_zeros += 1;
                } else {
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_zeros = 0;
                }
                // 连续32个0退出
                if consecutive_zeros == ID_LEN {
                    break 'outer;
                }
                // 每32位保存一个IDCODE
                if bit_count == ID_LEN {
                    // 连续32个1退出
                    if current_id == u32::MAX {
                        break 'outer;
                    }
                    idcodes.push(Some(current_id));
                    bit_count = 0;
                }
            }
        }
        // 退出Shift-DR状态
        drop(lock);
        self.goto_idle()?;
        Ok(idcodes)
    }
    pub fn write(&self, ir: &[u8], irlen: usize, dr: &[u8], drlen: usize) -> Result<(), FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();
        if !self.is_idle {
            cmd.jtag_any2idle();
        }
        cmd.jtag_idle2ir()
            .jtag_shift_write(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift_write(dr, drlen)
            .jtag_dr_exit2idle()
            .jtag_idle_cycle();
        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
    pub fn read(&self, ir: &[u8], irlen: usize, drlen: usize) -> Result<Vec<u8>, FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();
        if !self.is_idle {
            cmd.jtag_any2idle();
        }
        cmd.jtag_idle2ir()
            .jtag_shift_write(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift_read(drlen)
            .jtag_dr_exit2idle()
            .jtag_idle_cycle();
        let lock = self.mtx.lock().unwrap();
        let mut response = vec![0; cmd.read_len()];
        lock.write_read(cmd.as_slice(), &mut response)?;
        let len = JtagCmdBuilder::jtag_parse_single_shift(&mut response, drlen);

        if response.len() > len {
            response.pop();
        }
        Ok(response)
    }
    pub fn write_read(
        &self,
        ir: &[u8],
        irlen: usize,
        dr: &[u8],
        drlen: usize,
    ) -> Result<Vec<u8>, FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();
        if !self.is_idle {
            cmd.jtag_any2idle();
        }
        cmd.jtag_idle2ir()
            .jtag_shift_write(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift(dr, drlen)
            .jtag_dr_exit2idle()
            .jtag_idle_cycle();
        let lock = self.mtx.lock().unwrap();
        let mut response = vec![0; cmd.read_len()];
        lock.write_read(cmd.as_slice(), &mut response)?;
        let len = JtagCmdBuilder::jtag_parse_single_shift(&mut response, drlen);

        if response.len() > len {
            response.pop();
        }
        Ok(response)
    }
}
