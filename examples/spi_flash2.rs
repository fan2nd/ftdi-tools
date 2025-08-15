use std::{
    cell::RefCell,
    sync::{Arc, Mutex},
    time::Instant,
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
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let spi = RefCell::new(FtdiSpi::new(mtx.clone())?);
    let gpio = FtdiOutputPin::new(mtx, Pin::Lower(3))?;
    let mut flash_device = RefCellDevice::new_no_delay(&spi, gpio)?;
    let mut flash = Flash::new(&mut flash_device);
    let id = flash.read_id()?;
    println!("{id}");
    let param = flash.read_params()?.unwrap();
    println!("{param}");
    let data: Vec<_> = (0..param.capacity_bytes()).map(|x| x as u8).collect();
    flash.program_progress(0, &data, true)?;
    Ok(())
}
