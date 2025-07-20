use std::sync::{Arc, Mutex};

use anyhow::anyhow;
use ftdi_tools::{FtdiI2c, FtdiMpsse, list_all_device};
use lm75::Lm75;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut i2c = FtdiI2c::new(mtx)?;
    let addr_set = i2c.scan();
    println!("i2c detect:{:#?}", addr_set);
    let mut lm75 = Lm75::new(i2c, addr_set[0]);
    let temp = lm75.read_temperature().map_err(|e| {
        if let lm75::Error::I2C(inner) = e {
            anyhow!(inner)
        } else {
            anyhow!("lm75 internal error")
        }
    })?;
    println!("temperature:{}", temp);
    Ok(())
}
