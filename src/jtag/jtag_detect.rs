use crate::{FtdiMpsse, Pin, PinUse, ftdaye::FtdiError, mpsse_cmd::MpsseCmdBuilder};
use std::sync::{Arc, Mutex};

pub struct JtagDetectTdo {
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    tck: usize,
    tms: usize,
    has_direction: bool,
}
impl Drop for JtagDetectTdo {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        for i in 0..8 {
            lock.free_pin(Pin::Lower(i));
            if self.has_direction {
                lock.free_pin(Pin::Upper(i));
            }
        }
    }
}
impl JtagDetectTdo {
    /// Will use all lower pins
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
            has_direction: false,
        })
    }

    // 将JTAG状态机复位到Run-Test/Idle状态, 然后切换带shift-dr
    fn reset2dr(&self) -> Result<(), FtdiError> {
        let direction = 1 << self.tck | 1 << self.tms;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(1 << self.tck, direction);
        for _ in 0..5 {
            cmd
                // TMS1
                .set_gpio_lower(1 << self.tms, direction) // TCK to low
                .set_gpio_lower(1 << self.tck | 1 << self.tms, direction); // TCK to high
        }
        cmd
            // TMS0
            .set_gpio_lower(0, direction) // TCK to low
            .set_gpio_lower(1 << self.tck, direction) // TCK to high
            // TMS1
            .set_gpio_lower(1 << self.tms, direction) // TCK to low
            .set_gpio_lower(1 << self.tck | 1 << self.tms, direction) // TCK to high
            // TMS0
            .set_gpio_lower(0, direction) // TCK to low
            .set_gpio_lower(1 << self.tck, direction) // TCK to high
            .set_gpio_lower(0, direction) // TCK to low
            .set_gpio_lower(1 << self.tck, direction); // TCK to high
        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }

    fn shift_dr(&self, len: usize) -> Result<Vec<u8>, FtdiError> {
        let direction = 1 << self.tck | 1 << self.tms;
        let mut response = vec![0; len];
        let mut cmd = MpsseCmdBuilder::new();
        for _ in 0..len {
            cmd.set_gpio_lower(0, direction) // TCK to low
                .set_gpio_lower(1 << self.tck, direction) // TCK to high
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
    pub fn scan(&self) -> Result<Vec<Vec<Option<u32>>>, FtdiError> {
        const ID_LEN: usize = 32;
        self.reset2dr()?;

        // Shift 0s and read TDO until 32 consecutive 0s detected
        let mut idcodes = vec![Vec::new(); 8];
        let mut current_id = [0u32; 8];
        let mut bit_count = [0; 8];
        let mut consecutive_bypass = [0; 8];

        let read = self.shift_dr(ID_LEN * 2)?;
        // println!("read_buf{read:?}");
        for i in 0..8 {
            if i == self.tck || i == self.tms {
                continue;
            }
            let tdos: Vec<_> = read.iter().map(|&x| (x >> i) & 1 == 1).collect();

            for tdo_val in tdos {
                // Bypass detection - no device present
                if bit_count[i] == 0 && !tdo_val {
                    idcodes[i].push(None);
                    consecutive_bypass[i] += 1;
                } else {
                    // Accumulate IDCODE bits (MSB first)
                    current_id[i] = (current_id[i] >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count[i] += 1;
                    consecutive_bypass[i] = 0;
                }
                // Exit on 32 consecutive bypass bits
                if consecutive_bypass[i] == ID_LEN {
                    break;
                }
                // Store completed 32-bit IDCODE
                if bit_count[i] == ID_LEN {
                    // Terminate on invalid IDCODE (all 1s)
                    if current_id[i] == u32::MAX {
                        break;
                    }
                    idcodes[i].push(Some(current_id[i]));
                    bit_count[i] = 0;
                }
            }
        }
        Ok(idcodes)
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
        lock.free_pin(Pin::Lower(self.tck));
        lock.free_pin(Pin::Lower(self.tdi));
        lock.free_pin(Pin::Lower(self.tdo));
        lock.free_pin(Pin::Lower(self.tms));
    }
}
impl JtagDetectTdi {
    /// Only can use lower pins
    pub fn new(
        mtx: Arc<Mutex<FtdiMpsse>>,
        tck: usize,
        tdi: usize,
        tdo: usize,
        tms: usize,
    ) -> Result<Self, FtdiError> {
        let mut lock = mtx.lock().unwrap();
        lock.alloc_pin(Pin::Lower(tck), PinUse::Output);
        lock.alloc_pin(Pin::Lower(tdi), PinUse::Output);
        lock.alloc_pin(Pin::Lower(tdo), PinUse::Input);
        lock.alloc_pin(Pin::Lower(tms), PinUse::Output);
        // all pins default set to low
        lock.lower.direction |= 1 << tck | 1 << tdi | 1 << tms; // all pins default input, set tck/tdi/tms to output
        Ok(Self {
            mtx: mtx.clone(),
            tck,
            tdi,
            tdo,
            tms,
        })
    }

    // 辅助函数：产生时钟边沿并读取TDO
    fn clock_tck(&self, tms_val: u8, tdi_val: u8) -> Result<bool, FtdiError> {
        assert!(tms_val == 0 || tms_val == 1);
        assert!(tdi_val == 0 || tdi_val == 1);
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(
            lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
            lock.lower.direction,
        )
        .set_gpio_lower(
            lock.lower.value | 1 << self.tck | tdi_val << self.tdi | tms_val << self.tms,
            lock.lower.direction,
        )
        .gpio_lower()
        .set_gpio_lower(
            lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
            lock.lower.direction,
        );
        let mut response = [0u8];
        lock.write_read(cmd.as_slice(), &mut response)?;
        Ok(response[0] & (1 << self.tdo) != 0)
    }

    // 辅助函数：产生时钟边沿并读取TDO
    fn clock_tcks(&self, tms_val: u8, tdi_val: u8, count: usize) -> Result<Vec<bool>, FtdiError> {
        assert!(tms_val == 0 || tms_val == 1);
        assert!(tdi_val == 0 || tdi_val == 1);
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(
            lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
            lock.lower.direction,
        );
        for _ in 0..count {
            cmd.set_gpio_lower(
                lock.lower.value | 1 << self.tck | tdi_val << self.tdi | tms_val << self.tms,
                lock.lower.direction,
            ) // set tck to high
            .gpio_lower()
            .set_gpio_lower(
                lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
                lock.lower.direction,
            ); // set tck to low
        }
        let mut response = vec![0; count];
        lock.write_read(cmd.as_slice(), &mut response)?;
        Ok(response
            .into_iter()
            .map(|x| x & 1 << self.tdo != 0)
            .collect())
    }

    // 将JTAG状态机复位到Run-Test/Idle状态
    fn goto_idle(&self) -> Result<(), FtdiError> {
        // 发送5个TMS=1复位状态机 (Test-Logic-Reset)
        self.clock_tcks(1, 1, 5)?;
        // 进入Run-Test/Idle: TMS=0 -> Run-Test/Idle
        self.clock_tck(0, 1)?;
        // 保持Run-Test/Idle (TMS=0)
        self.clock_tck(0, 1)?;
        Ok(())
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
    pub fn scan_with(&self, tdi_val: u8) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        self.goto_idle()?;

        // Enter Shift-DR state
        self.clock_tck(1, 1)?; // Select-DR-Scan
        self.clock_tck(0, 1)?; // Capture-DR
        self.clock_tck(0, 1)?; // Shift-DR

        // Shift TDI value and read TDO until 32 consecutive 0s detected
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_bypass = 0;

        'outer: loop {
            let tdos = self.clock_tcks(0, tdi_val, ID_LEN)?; // Shift in tdi_val
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
                // Store completed 32-bit IDCODE
                if bit_count == ID_LEN {
                    // Terminate on invalid IDCODE (all 1s)
                    if current_id == u32::MAX {
                        break 'outer;
                    }
                    idcodes.push(Some(current_id));
                    bit_count = 0;
                }
            }
        }

        // Exit Shift-DR state
        self.clock_tck(1, 0)?; // Exit1-DR
        self.clock_tck(1, 0)?; // Update-DR
        self.clock_tck(0, 0)?; // Return to Run-Test/Idle

        Ok(idcodes)
    }
}
