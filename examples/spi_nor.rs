use std::sync::{Arc, Mutex};

use eh1::spi::SpiBus;
use ftdi_tools::{FtdiMpsse, OutputPin, Pin, Spi, list_all_device};

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut spi = Spi::new(mtx.clone())?;
    let gpio = OutputPin::new(mtx, Pin::Lower(3))?;
    gpio.set(true)?;
    {
        let read_buf = &mut [0; 3];
        gpio.set(false)?;
        spi.write(&[0x9f])?;
        spi.read(read_buf)?;
        gpio.set(true)?;
        println!("Manufacturer ID:{:#x?}", read_buf[0]);
        println!(
            "Device ID:{:#x?}",
            u16::from_be_bytes([read_buf[1], read_buf[2]])
        );
    }
    Ok(())
}
