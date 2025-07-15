use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_tools::{FtdiMpsse, JtagDetectTdi, list_all_device};
use itertools::Itertools;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let now = Instant::now();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let pins = [0, 1, 2, 3, 4, 5, 6, 7];
    for couple in pins.into_iter().permutations(4) {
        let softjtag = JtagDetectTdi::new(mtx.clone(), couple[0], couple[1], couple[2], couple[3])?;
        let ids_scan1 = softjtag.scan_with(1)?;
        if ids_scan1.iter().any(Option::is_some) {
            print!(
                "testing: [tck:{:?},tdi:{:?},tdo:{:?},tms:{:?}]",
                couple[0], couple[1], couple[2], couple[3],
            );
            let ids_scan0 = softjtag.scan_with(0)?;
            if ids_scan0.len() <= ids_scan1.len() {
                println!("!!!!!! tdi is not correct");
            } else {
                println!("!!!!!! Found Devices:{ids_scan1:x?}");
            }
        }
    }
    println!("Finish Detect Using {:?}", now.elapsed());
    Ok(())
}
