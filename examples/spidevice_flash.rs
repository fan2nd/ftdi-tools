//! SPI Flash 存储器操作示例 (使用 SPI Device 模式)
//!
//! 此示例演示如何使用 FTDI 芯片的 FtdiSpiDevice 直接与 Flash 存储器通信。
//! 与 spibus_flash.rs 的区别是这里使用内建的片选控制，更加简单直接。
//!
//! FtdiSpiDevice vs FtdiSpi + GPIO:
//! - FtdiSpiDevice: 内置片选控制，使用更简单，适合单一 SPI 设备
//! - FtdiSpi + GPIO: 手动片选控制，灵活性更高，适合多设备共享 SPI 总线
//!
//! 支持的 Flash 类型:
//! - NOR Flash: W25Q32/64/128, MX25L, AT25SF, S25FL 等
//! - 支持标准 JEDEC 命令集
//!
//! 硬件连接 (默认引脚配置):
//! - SCK: FTDI AD0 (Pin 0) - 串行时钟
//! - MOSI: FTDI AD1 (Pin 1) - 主机输出从机输入
//! - MISO: FTDI AD2 (Pin 2) - 主机输入从机输出  
//! - CS: FTDI AD3 (Pin 3) - 片选信号 (自动控制)
//!
//! 警告: 此示例会擦除并重新编程整个 Flash!
//!
//! 运行方式:
//! ```bash
//! RUST_LOG=info cargo run --example spidevice_flash
//! ```

use std::sync::{Arc, Mutex};

use anyhow::anyhow;
use eh1::spi::SpiDevice;
use ftdi_tools::{Interface, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpiDevice};
use spi_flash::{Error, Flash, FlashAccess};

/// Flash 设备适配器结构体
///
/// 将任意实现了 SpiDevice trait 的类型包装成 FlashAccess trait
struct FlashDevice<T>(T);

/// 为 FlashDevice 实现 FlashAccess trait  
///
/// 这个适配器模式允许我们使用 embedded-hal 的 SpiDevice 与 spi-flash 库一起工作
impl<T: SpiDevice> FlashAccess for FlashDevice<T> {
    type Error = Error;

    /// 执行 SPI 数据交换
    ///
    /// # 参数
    /// * `data` - 发送给 Flash 的命令和数据
    ///
    /// # 返回值  
    /// * `Ok(Vec<u8>)` - Flash 返回的数据
    /// * `Err(Error)` - 通信错误
    fn exchange(&mut self, data: &[u8]) -> core::result::Result<Vec<u8>, Self::Error> {
        // 创建接收缓冲区，长度与发送数据相同
        let mut result = vec![0; data.len()];
        // 执行 SPI 事务传输 (自动处理片选信号)
        self.0
            .transfer(&mut result, data)
            .map_err(|_| Error::Access(anyhow!("SPI device transfer failed")))?;
        Ok(result)
    }
}

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
    let spidevice = FtdiSpiDevice::new(mtx)?;

    // 将 SPI 设备包装成 Flash 访问接口
    let mut flash_device = FlashDevice(spidevice);

    // 初始化 Flash 控制器实例
    let mut flash = Flash::new(&mut flash_device);

    // 读取 Flash 设备的 JEDEC 标识符
    // JEDEC ID 是一个 3 字节的标识符，包含厂商 ID、设备类型和容量信息
    let id = flash.read_id()?;
    println!("Flash JEDEC ID: {}", id);

    // 读取 Flash 设备的详细参数信息
    // 包括容量、页大小、块大小等重要参数
    let param = flash.read_params()?.unwrap();
    println!("Flash Parameters: {}", param);

    // 生成测试数据
    // 创建一个从 0 到 Flash 容量-1 的字节序列
    // 注意: 对于大容量 Flash，这可能会使用大量内存!
    let data: Vec<_> = (0..param.capacity_bytes()).map(|x| x as u8).collect();

    // 执行全片编程操作
    // program_progress(start_addr, data, verify)
    // - start_addr: 起始地址 (0 = 从头开始)
    // - data: 要编程的数据
    // - verify: 是否需要校验
    flash.program_progress(0, &data, true)?;
    Ok(())
}
