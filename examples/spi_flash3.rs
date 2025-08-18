use ftdi_tools::{Interface, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpiDevice};
use spi_flash2::Flash;
use std::sync::{Arc, Mutex};

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut spi_device = FtdiSpiDevice::new(mtx)?;
    let mut flash = Flash::new(&mut spi_device);
    let id = flash.read_id()?;
    println!("{id}");
    let param = flash.read_params()?.unwrap();
    println!("{param}");
    let data: Vec<_> = (0..param.capacity_bytes()).map(|x| x as u8).collect();
    flash.program_progress(0, &data, true)?;
    Ok(())
}
