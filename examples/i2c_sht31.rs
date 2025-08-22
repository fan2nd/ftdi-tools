//! I2C SHT31 温湿度传感器读取示例
//!
//! 此示例演示如何使用 FTDI 芯片通过 I2C 接口与 SHT31 温湿度传感器通信。
//! SHT31 是 Sensirion 公司生产的高精度数字温湿度传感器。
//!
//! 传感器特性:
//! - 温度精度: ±0.2°C (典型值)
//! - 湿度精度: ±2%RH (典型值)
//! - I2C 地址: 0x44 (ADDR 引脚接 GND) 或 0x45 (ADDR 引脚接 VDD)
//!
//! 硬件连接:
//! - SCL: FTDI AD0 (Pin 0)
//! - SDA_O (数据线): FTDI AD1
//! - SDA_I (数据线): FTDI AD2(与AD1短接)
//! - VDD: 2.4V - 5.5V
//! - GND: 接地
//!
//! 运行方式:
//! ```bash
//! RUST_LOG=info cargo run --example i2c_sht31
//! ```

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use sht31::prelude::*;

/// 主函数 - 程序入口点
///
/// 执行流程:
/// 1. 初始化日志系统
/// 2. 连接 FTDI 设备并配置 I2C
/// 3. 启用快速模式提高通信效率
/// 4. 扫描 I2C 设备
/// 5. 配置 SHT31 传感器参数
/// 6. 执行单次测量并读取结果
fn main() -> anyhow::Result<()> {
    // 初始化日志系统，支持通过 RUST_LOG 环境变量控制输出级别
    env_logger::init();
    
    // 获取系统中所有可用的 FTDI 设备列表
    let devices = list_all_device();
    // 检查是否找到可用的 FTDI 设备
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    
    // 打开第一个可用的 FTDI 设备的第一个接口
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    // 将 MPSSE 控制器包装在线程安全的互斥锁中
    let mtx = Arc::new(Mutex::new(mpsse));
    
    // 创建 I2C 主控制器，默认时钟频率 100kHz
    let mut i2c = FtdiI2c::new(mtx)?;
    // 启用快速模式，提高数据传输效率
    // 快速模式会将多个操作打包在一个 MPSSE 命令中
    i2c.enbale_fast(true);
    
    // 扫描 I2C 总线以查找连接的设备
    let addr_set = i2c.scan();
    // 输出扫描结果，显示所有在线设备的 I2C 地址
    println!("i2c detect:{:#x?}", addr_set);
    
    // 创建 SHT31 传感器实例，配置为单次测量模式
    let mut sht = SHT31::single_shot(i2c, SingleShot::new())
        .with_accuracy(Accuracy::High)           // 设置为高精度模式
        .with_unit(TemperatureUnit::Celsius)      // 温度单位设置为摄氏度
        .with_address(DeviceAddr::AD0);           // 使用默认地址 0x44

    // 启动测量过程，这个操作是异步的
    // SHT31 需要一定时间来完成温湿度测量
    sht.measure()?;

    // 读取温湿度数据并循环显示
    // 此处使用 while let 来持续读取数据直到发生错误
    while let Ok(reading) = sht.read() {
        // reading 包含温度和湿度信息
        println!("{reading:?}");
    }
    Ok(())
}
