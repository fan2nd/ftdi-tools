use std::sync::{Arc, Mutex};

use eh1::{digital::OutputPin, spi::SpiBus};
use ftdi_tools::{
    gpio::FtdiOutputPin,
    list_all_device,
    mpsse::{FtdiMpsse, Pin},
    spi::FtdiSpi,
};

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut spi = FtdiSpi::new(mtx.clone())?;
    let mut gpio = FtdiOutputPin::new(mtx, Pin::Lower(3))?;
    gpio.set_high()?;
    {
        let read_buf = &mut [0; 3];
        gpio.set_low()?;
        spi.write(&[0x9f])?;
        spi.read(read_buf)?;
        gpio.set_high()?;
        println!("Manufacturer ID:{:#x?}", read_buf[0]);
        println!(
            "Device ID:{:#x?}",
            u16::from_be_bytes([read_buf[1], read_buf[2]])
        );
    }
    Ok(())
}
