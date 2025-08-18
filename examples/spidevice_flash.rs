use std::sync::{Arc, Mutex};

use anyhow::anyhow;
use eh1::spi::SpiDevice;
use ftdi_tools::{Interface, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpiDevice};
use spi_flash::{Error, Flash, FlashAccess};

struct FlashDevice<T>(T);
impl<T: SpiDevice> FlashAccess for FlashDevice<T> {
    type Error = Error;
    fn write(&mut self, data: &[u8]) -> core::result::Result<(), Self::Error> {
        self.0
            .write(data)
            .map_err(|_| Error::Access(anyhow!("fuck rust error")))?;
        Ok(())
    }
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
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let spidevice = FtdiSpiDevice::new(mtx)?;
    let mut flash_device = FlashDevice(spidevice);
    let mut flash = Flash::new(&mut flash_device);
    let id = flash.read_id()?;
    println!("{id}");
    let param = flash.read_params()?.unwrap();
    println!("{param}");
    let data: Vec<_> = (0..param.capacity_bytes()).map(|x| x as u8).collect();
    flash.program_progress(0, &data, true)?;
    Ok(())
}
