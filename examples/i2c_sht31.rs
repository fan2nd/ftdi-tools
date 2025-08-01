use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use ftdi_tools::{FtdiI2c, FtdiMpsse, list_all_device};
use sht31::prelude::*;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut i2c = FtdiI2c::new(mtx)?;
    i2c.enbale_fast(true);
    let addr_set = i2c.scan();
    println!("i2c detect:{:#x?}", addr_set);
    // Makes the sensor acquire the data at 4 Hz
    let mut sht = SHT31::single_shot(i2c, SingleShot::new())
        .with_accuracy(Accuracy::High)
        .with_unit(TemperatureUnit::Celsius)
        .with_address(DeviceAddr::AD0);

    // Start measuring before you need the reading,
    // this is more efficient than waiting for readings
    sht.measure()?;

    std::thread::sleep(Duration::from_secs(1));

    while let Ok(reading) = sht.read() {
        println!("{reading:?}");
    }
    Ok(())
}
