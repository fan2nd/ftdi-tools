use std::sync::{Arc, Mutex};

use ftdi_tools::{
    list_all_device,
    mpsse::FtdiMpsse,
    swd::{FtdiSwd, SwdAddr},
};

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    let mtx = Arc::new(Mutex::new(mpsse));
    let swd = FtdiSwd::new(mtx)?;
    swd.enable()?;
    let idcode = swd.read(SwdAddr::Dp(0))?;
    // g431cbu6: 0x2BA01477; according to rm0440-47.8.5
    println!("idcode:{idcode:#x?}");
    Ok(())
}
