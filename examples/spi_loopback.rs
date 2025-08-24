use std::sync::{Arc, Mutex};

use eh1::spi::{Operation, SpiDevice};
use ftdi_tools::{Interface, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpiDevice};

fn main() -> anyhow::Result<()> {
    // 初始化日志输出系统
    env_logger::init();

    // 扫描并获取所有连接的 FTDI 设备
    let devices = list_all_device();
    // 验证系统中存在可用的 FTDI 设备
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    // 打开第一个 FTDI 设备的接口 A
    // 接口 A 通常是主接口，支持全部 MPSSE 功能
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    // 使用线程安全的 Arc<Mutex<>> 包装 MPSSE 控制器
    let mtx = Arc::new(Mutex::new(mpsse));

    // 创建 FtdiSpiDevice 实例
    // 这个设备封装了 SPI 总线和片选控制，提供了完整的 SpiDevice 实现
    let mut spidevice = FtdiSpiDevice::new(mtx)?;

    let max = 600000;
    let data: Vec<_> = (0..max).map(|x| (x * 7 + 3) as u8).collect();
    let mut read_back1 = vec![0; 16635];
    let mut read_back2 = vec![0; max - 16635];
    spidevice.transaction(&mut [
        Operation::Transfer(&mut read_back1, &data[0..16635]),
        Operation::Transfer(&mut read_back2, &data[16635..max]),
    ])?;

    read_back1.extend(read_back2);
    read_back1
        .into_iter()
        .zip(data)
        .for_each(|x| assert_eq!(x.0, x.1));
    Ok(())
}
