use std::sync::{Arc, Mutex};

use ftdi_tools::{FtdiMpsse, I2c, list_all_device};
use lm75::Lm75;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut i2c = I2c::new(mtx)?;
    let addr_set = i2c.scan();
    println!("i2c detect:{:#?}", addr_set);
    let mut lm75 = Lm75::new(i2c, addr_set[0]);
    println!("temperature:{}", lm75.read_temperature().unwrap());
    Ok(())
}
