//! I2C LM75 温度传感器读取示例
//! 
//! 此示例演示如何使用 FTDI 芯片通过 I2C 接口与 LM75 温度传感器通信。
//! LM75 是一种广泛使用的数字温度传感器，支持 I2C 接口。
//! 
//! 硬件连接:
//! - SCL (时钟线): FTDI AD0
//! - SDA_O (数据线): FTDI AD1
//! - SDA_I (数据线): FTDI AD2(与AD1短接)
//! - VCC: 3.3V 或 5V
//! - GND: 接地
//! 
//! 运行方式:
//! ```bash
//! RUST_LOG=debug cargo run --example i2c_lm75
//! ```

use std::sync::{Arc, Mutex};

use anyhow::anyhow;
use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use lm75::Lm75;

/// 主函数 - 程序入口点
/// 
/// 执行流程:
/// 1. 初始化日志系统
/// 2. 扫描并连接 FTDI 设备
/// 3. 配置 I2C 接口
/// 4. 扫描 I2C 总线上的设备
/// 5. 初始化 LM75 传感器
/// 6. 读取并显示温度数据
fn main() -> anyhow::Result<()> {
    // 启用环境变量控制的日志输出
    // 使用 RUST_LOG=debug 可以查看详细的调试信息
    env_logger::init();
    
    // 扫描系统中所有可用的 FTDI 设备
    let devices = list_all_device();
    // 确保至少找到一个 FTDI 设备，否则程序会 panic
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    
    // 打开第一个 FTDI 设备的第一个接口，初始化 MPSSE 模式
    // MPSSE (Multi-Protocol Synchronous Serial Engine) 支持 SPI/I2C/JTAG 等协议
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    // 使用 Arc<Mutex<>> 包装以支持线程安全的共享访问
    let mtx = Arc::new(Mutex::new(mpsse));
    
    // 创建 I2C 主控制器实例，默认配置为 100kHz
    let mut i2c = FtdiI2c::new(mtx)?;
    
    // 扫描 I2C 总线上的所有设备地址 (0x00 - 0x7F)
    // 这个操作会对每个地址发送 START + 地址 + 读/写位 + ACK/NACK
    let addr_set = i2c.scan();
    // 以十六进制格式显示扫描到的设备地址
    println!("i2c detect:{:#x?}", addr_set);
    
    // 使用扫描到的第一个设备地址创建 LM75 传感器实例
    // LM75 默认地址通常为 0x48-0x4F，取决于 A0/A1/A2 引脚的连接
    let mut lm75 = Lm75::new(i2c, addr_set[0]);
    
    // 读取温度数据，返回值为摄氏度
    // LM75 的温度分辨率为 0.5°C，温度范围 -55°C 到 +125°C
    let temp = lm75.read_temperature().map_err(|e| {
        // 将 lm75 库的错误类型转换为 anyhow 错误类型
        if let lm75::Error::I2C(inner) = e {
            anyhow!(inner)
        } else {
            anyhow!("lm75 internal error")
        }
    })?;
    
    // 显示温度读数，单位为摄氏度
    println!("temperature:{}", temp);
    Ok(())
}
