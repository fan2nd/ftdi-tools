use std::{
    cell::RefCell,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::anyhow;
use eh1::digital::OutputPin;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use embedded_hal_bus::spi::RefCellDevice;
use ftdi_tools::{
    Interface, Pin, delay::Delay, gpio::FtdiOutputPin, list_all_device, mpsse::FtdiMpsse,
    spi::FtdiSpiTx,
};
use mipidsi::{
    Builder, TestImage,
    interface::SpiInterface,
    models::ST7789,
    options::{ColorInversion, ColorOrder},
};

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
    let spibus = RefCell::new(FtdiSpiTx::new(mtx.clone())?);
    let cs = FtdiOutputPin::new(mtx.clone(), Pin::Lower(3))?;
    let spidevice = RefCellDevice::new_no_delay(&spibus, cs)?;
    let rst = FtdiOutputPin::new(mtx.clone(), Pin::Lower(4))?;
    let dc = FtdiOutputPin::new(mtx.clone(), Pin::Lower(5))?;
    let mut blk = FtdiOutputPin::new(mtx.clone(), Pin::Lower(6))?;
    blk.set_high()?;

    let buffer = &mut [0; 240 * 280 * 2];
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
    Rectangle::new(Point { x: 75, y: 90 }, Size::new(100, 100))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();
    std::thread::sleep(Duration::from_millis(500));
    TestImage::new().draw(&mut display).unwrap();

    // Create a new character style
    let style = MonoTextStyle::new(&FONT_10X20, RgbColor::BLACK);
    Text::new("Hello Rust!", Point { x: 26, y: 120 }, style)
        .draw(&mut display)
        .unwrap();
    Ok(())
}
