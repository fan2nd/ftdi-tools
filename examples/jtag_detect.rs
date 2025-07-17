use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_tools::{FtdiMpsse, JtagDetectTdi, JtagDetectTdo, Pin, list_all_device};
use itertools::Itertools;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let now = Instant::now();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut notdi = Vec::new();
    // find tdo
    for couple in (0..8).into_iter().permutations(2) {
        let tck = couple[0];
        let tms = couple[1];
        let jtag = JtagDetectTdo::new(mtx.clone(), tck, tms)?;
        // you can add code here to control level translation chip
        // tck and tms output, other input
        let tdo_pins = jtag.scan()?;
        for tdo in tdo_pins.into_iter() {
            if let Pin::Lower(tdo_idx) = tdo {
                notdi.push((tck, tms, tdo_idx));
            }
        }
    }
    // find tdi
    for (tck, tms, tdo) in notdi {
        // you can add code here to control level translation chip
        // tdo input, other output
        for tdi in 0..8 {
            if tdi == tck || tdi == tms || tdi == tdo {
                continue;
            }
            let jtag = JtagDetectTdi::new(mtx.clone(), tck, tdi, tdo, tms)?;
            let ids_scan1 = jtag.scan_with(true)?;
            let ids_scan0 = jtag.scan_with(false)?;
            // Scanning with 1 will give you 32 more bypasses than scanning with 0
            if ids_scan0.len() - ids_scan1.len() == 32 {
                println!("!!!!!! Pins:tck[{tck}],tdi[{tdi}],tdo[{tdo}],tms[{tms}]");
                println!("!!!!!! Found Devices:{ids_scan1:x?}");
            }
        }
    }
    println!("Finish Detect Using {:?}", now.elapsed());
    Ok(())
}
