use crate::{FtdiMpsse, Pin, PinUse, ftdaye::FtdiError, mpsse_cmd::MpsseCmdBuilder};
use std::sync::{Arc, Mutex, MutexGuard};

pub struct JtagDetectTdo {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    tck: usize,
    tms: usize,
}
impl Drop for JtagDetectTdo {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        for i in 0..8 {
            lock.free_pin(Pin::Lower(i));
        }
    }
}
impl JtagDetectTdo {
    /// Will use all lower pins as tdi(except tck & tms)
    ///
    /// If you want to use the level translation chip, please use [`crate::FtdiOutputPin`] to control.
    ///
    /// Parameters:
    ///
    /// tck & tms are all lower pins index
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, tck: usize, tms: usize) -> Result<Self, FtdiError> {
        let mut lock = mtx.lock().unwrap();
        for i in 0..8 {
            if i == tck || i == tms {
                lock.alloc_pin(Pin::Lower(i), PinUse::Output);
            } else {
                lock.alloc_pin(Pin::Lower(i), PinUse::Input);
            }
        }
        // all pins default set to low
        Ok(Self {
            mtx: mtx.clone(),
            tck,
            tms,
        })
    }
    fn shift_dr(&self, len: usize) -> Result<Vec<u8>, FtdiError> {
        let direction = 1 << self.tck | 1 << self.tms;
        let mut response = vec![0; len];
        let mut cmd = MpsseCmdBuilder::new();
        for _ in 0..len {
            cmd.set_gpio_lower(0, direction) // TCK0,TMS0,
                .set_gpio_lower(1 << self.tck, direction) // TCK1,TMS0,
                .gpio_lower();
        }
        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut response)?;
        Ok(response)
    }
    /// Scans JTAG chain to identify connected devices through TDO pins
    ///
    /// # Returns
    /// Result containing 8-element vector (one per TDO pin) of detected IDCODE vectors
    /// Each inner vector contains Option<u32> where:
    /// - Some(u32): Valid 32-bit IDCODE detected
    /// - None: Bypass detected
    /// - Empty: No devices detected on that pin
    ///
    /// # Protocol Flow
    /// 1. Resets JTAG state machine to Shift-DR
    /// 2. Shifts 64 bits (2*ID_LEN) through scan chain
    /// 3. Analyzes TDO responses from all non-TCK/TMS pins
    /// 4. Detects IDCODEs by accumulating 32-bit sequences
    /// 5. Terminates on 32 consecutive bypass bits or invalid IDCODE
    pub fn scan(&self) -> Result<Vec<Pin>, FtdiError> {
        const ID_LEN: usize = 32;
        let mut tdo_pins = Vec::new();
        let lock = self.mtx.lock().unwrap();
        reset2dr(lock, self.tck, self.tms)?;
        let read = self.shift_dr(ID_LEN * 2)?;
        // println!("read_buf{read:?}");
        for i in 0..8 {
            if i == self.tck || i == self.tms {
                continue;
            }
            let mut current_id = 0;
            let mut bit_count = 0;
            let mut consecutive_bypass = 0;
            let tdos: Vec<_> = read.iter().map(|&x| (x >> i) & 1 == 1).collect();

            for tdo_val in tdos {
                // Bypass detection - no device present
                if bit_count == 0 && !tdo_val {
                    consecutive_bypass += 1;
                } else {
                    // Accumulate IDCODE bits (LSB first)
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_bypass = 0;
                }
                // Exit on 32 consecutive bypass bits
                if consecutive_bypass == ID_LEN {
                    break;
                }
                // Store completed 32-bit IDCODE
                if bit_count == ID_LEN {
                    // Terminate on invalid IDCODE (all 1s)
                    if current_id != u32::MAX {
                        tdo_pins.push(Pin::Lower(i));
                    }
                    break;
                }
            }
        }
        Ok(tdo_pins)
    }
}

pub struct JtagDetectTdi {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    tck: usize,
    tdi: usize,
    tdo: usize,
    tms: usize,
}
impl Drop for JtagDetectTdi {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        for i in 0..8 {
            lock.free_pin(Pin::Lower(i));
        }
    }
}
impl JtagDetectTdi {
    /// If you want to use the level translation chip, please use [`crate::FtdiOutputPin`] to control.
    /// Parameters:
    ///
    /// tck & tdi & tdo & tms are all lower pins index
    pub fn new(
        mtx: Arc<Mutex<FtdiMpsse>>,
        tck: usize,
        tdi: usize,
        tdo: usize,
        tms: usize,
    ) -> Result<Self, FtdiError> {
        let mut lock = mtx.lock().unwrap();
        for i in 0..8 {
            if i == tdo {
                lock.alloc_pin(Pin::Lower(i), PinUse::Input);
            } else {
                lock.alloc_pin(Pin::Lower(i), PinUse::Output);
            }
        }
        Ok(Self {
            mtx: mtx.clone(),
            tck,
            tdi,
            tdo,
            tms,
        })
    }
    fn shift_dr(&self, tdi_value: bool, len: usize) -> Result<Vec<bool>, FtdiError> {
        let direction = !(1 << self.tdo); // all output except tdo
        let tdi_mask = if tdi_value { 1 << self.tdi } else { 0 };
        let mut response = vec![0; len];
        let mut cmd = MpsseCmdBuilder::new();
        for _ in 0..len {
            cmd.set_gpio_lower(tdi_mask, direction) // TCK0,TMS0,
                .set_gpio_lower(tdi_mask | 1 << self.tck, direction) // TCK1,TMS0,
                .gpio_lower();
        }
        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut response)?;
        Ok(response
            .into_iter()
            .map(|x| x & (1 << self.tdo) != 0)
            .collect())
    }
    /// Scans JTAG chain to identify connected devices with specified TDI value
    ///
    /// # Arguments
    /// * `tdi_val` - TDI pin value to drive during scanning (0 or 1)
    ///
    /// # Returns
    /// Result containing vector of detected IDCODEs where:
    /// - Some(u32): Valid 32-bit IDCODE detected
    /// - None: Bypass detected
    /// - Empty: No devices detected
    ///
    /// # Protocol Flow
    /// 1. Resets JTAG state machine to Run-Test/Idle
    /// 2. Enters Shift-DR state through Select-DR-Scan → Capture-DR → Shift-DR
    /// 3. Continuously shifts specified TDI value while monitoring TDO
    /// 4. Detects IDCODEs by accumulating 32-bit sequences
    /// 5. Terminates on 32 consecutive bypass bits or invalid IDCODE
    /// 6. Exits to Run-Test/Idle through Exit1-DR → Update-DR
    pub fn scan_with(&self, tdi_val: bool) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        // Shift TDI value and read TDO until 32 consecutive 0s detected
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_bypass = 0;

        let lock = self.mtx.lock().unwrap();
        reset2dr(lock, self.tck, self.tms)?;

        'outer: loop {
            let tdos: Vec<_> = self.shift_dr(tdi_val, ID_LEN * 2)?;
            for tdo_val in tdos {
                // Bypass detection - no device present
                if bit_count == 0 && !tdo_val {
                    idcodes.push(None);
                    consecutive_bypass += 1;
                } else {
                    // Accumulate IDCODE bits (MSB first)
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_bypass = 0;
                }
                // Exit on 32 consecutive bypass bits
                if consecutive_bypass == ID_LEN {
                    break 'outer;
                }
                // Exit on 32 consecutive one bits or this idcode is valid
                if bit_count == ID_LEN {
                    if current_id == u32::MAX {
                        break 'outer;
                    }
                    idcodes.push(Some(current_id));
                }
            }
        }

        Ok(idcodes)
    }
}
// 将JTAG状态机复位到Run-Test/Idle状态, 然后切换到shift-dr
fn reset2dr(lock: MutexGuard<FtdiMpsse>, tck: usize, tms: usize) -> Result<(), FtdiError> {
    let tck_mask = 1 << tck;
    let tms_mask = 1 << tms;
    let direction = tck_mask | tms_mask;
    let mut cmd = MpsseCmdBuilder::new();
    cmd.set_gpio_lower(tck_mask, direction);
    for _ in 0..5 {
        cmd
            // TMS1
            .set_gpio_lower(tms_mask, direction) // TCK to low
            .set_gpio_lower(tck_mask | tms_mask, direction); // TCK to high
    }
    cmd
        // TMS0
        .set_gpio_lower(0, direction) // TCK to low
        .set_gpio_lower(tck_mask, direction) // TCK to high
        // TMS1
        .set_gpio_lower(tms_mask, direction) // TCK to low
        .set_gpio_lower(tck_mask | tms_mask, direction) // TCK to high
        // TMS0
        .set_gpio_lower(0, direction) // TCK to low
        .set_gpio_lower(tck_mask, direction) // TCK to high
        .set_gpio_lower(0, direction) // TCK to low
        .set_gpio_lower(tck_mask, direction); // TCK to high
    lock.write_read(cmd.as_slice(), &mut [])?;
    Ok(())
}
