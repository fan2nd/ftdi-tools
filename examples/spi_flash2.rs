use std::{
    cell::RefCell,
    sync::{Arc, Mutex},
};

use embedded_hal_bus::spi::RefCellDevice;
use ftdi_tools::{
    Interface,
    gpio::FtdiOutputPin,
    list_all_device,
    mpsse::{FtdiMpsse, Pin},
    spi::FtdiSpi,
};
use spi_flash2::Flash;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::B)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let spi = RefCell::new(FtdiSpi::new(mtx.clone())?);
    let gpio = FtdiOutputPin::new(mtx, Pin::Lower(3))?;
    let mut flash_device = RefCellDevice::new_no_delay(&spi, gpio)?;
    let mut flash = Flash::new(&mut flash_device);
    let param = flash.read_params().unwrap().unwrap();
    println!("{param:#x?}");
    Ok(())
}
