use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::anyhow;
use eh1::digital::OutputPin;
use embedded_graphics::{
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};
use ftdi_tools::{
    Interface, Pin, delay::Delay, gpio::FtdiOutputPin, list_all_device, mpsse::FtdiMpsse,
    spi::FtdiSpiDevice,
};
use mipidsi::{
    Builder, TestImage,
    interface::SpiInterface,
    models::ST7789,
    options::{ColorInversion, ColorOrder},
};

/// 主函数 - SPI Flash 操作程序入口
///
/// 此函数实现了使用 FtdiSpiDevice 的 Flash 操作流程:
/// 1. 初始化 FTDI 设备和 SPI 接口
/// 2. 创建具有内建片选控制的 SPI 设备
/// 3. 读取 Flash 设备信息  
/// 4. 执行全片数据编程
///
/// 返回值: anyhow::Result<()> - 操作结果
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
    let spidevice = FtdiSpiDevice::new(mtx.clone())?;
    let rst = FtdiOutputPin::new(mtx.clone(), Pin::Lower(4))?;
    let dc = FtdiOutputPin::new(mtx.clone(), Pin::Lower(5))?;
    let mut blk = FtdiOutputPin::new(mtx.clone(), Pin::Lower(6))?;
    blk.set_high()?;
    let buffer = &mut [0; 256 * 256 * 3];
    let interface = SpiInterface::new(spidevice, dc, buffer);

    let mut display = Builder::new(ST7789, interface)
        .reset_pin(rst)
        .display_size(240, 280)
        .display_offset(0, 20)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Rgb)
        .init(&mut Delay)
        .map_err(|x| anyhow!("{x:?}"))?;
    display.clear(RgbColor::RED).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    display.clear(RgbColor::GREEN).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    display.clear(RgbColor::BLUE).unwrap();

    let style = PrimitiveStyleBuilder::new()
        .fill_color(RgbColor::BLACK)
        .build();
    Rectangle::new(Point { x: 50, y: 50 }, Size::new(50, 50))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();
    std::thread::sleep(Duration::from_millis(500));
    TestImage::new().draw(&mut display).unwrap();
    Ok(())
}
