use std::{
    cell::RefCell,
    sync::{Arc, Mutex},
};

use anyhow::anyhow;
use eh1::spi::SpiDevice;
use embedded_hal_bus::spi::RefCellDevice;
use ftdi_tools::{
    Interface, Pin, gpio::FtdiOutputPin, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpi,
};
use spi_flash::{Error, Flash, FlashAccess};

struct FlashDevice<T>(T);
impl<T: SpiDevice> FlashAccess for FlashDevice<T> {
    type Error = Error;
    fn exchange(&mut self, data: &[u8]) -> core::result::Result<Vec<u8>, Self::Error> {
        let mut result = vec![0; data.len()];
        self.0
            .transfer(&mut result, data)
            .map_err(|_| Error::Access(anyhow!("fuck rust error")))?;
        Ok(result)
    }
}

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::B)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let spi = RefCell::new(FtdiSpi::new(mtx.clone())?);
    let gpio = FtdiOutputPin::new(mtx, Pin::Lower(3))?;
    let mut flash_device = FlashDevice(RefCellDevice::new_no_delay(&spi, gpio)?);
    let mut flash = Flash::new(&mut flash_device);
    let id = flash.read_id()?;
    println!("{id}");
    let param = flash.read_params()?.unwrap();
    println!("{param}");
    Ok(())
}
