//! JTAG 链扫描示例
//!
//! 此示例演示如何使用 FTDI 芯片通过已知的 JTAG 引脚配置扫描 JTAG 链。
//! 与 jtag_detect.rs 不同，这个示例假设 JTAG 引脚已经知道或已经配置好。
//!
//! JTAG 链扫描原理:
//! - JTAG 设备以菊花链形式连接，数据从 TDI 进入，经过所有设备后从 TDO 输出
//! - 通过发送特定的命令序列，可以读取链上每个设备的 ID
//! - 每个 JTAG 设备都有一个唯一的 32 位 IDCODE
//!
//! 默认引脚配置:
//! - TCK: FTDI AD0 (Pin 0) - 时钟信号
//! - TDI: FTDI AD1 (Pin 1) - 数据输入
//! - TDO: FTDI AD2 (Pin 2) - 数据输出  
//! - TMS: FTDI AD3 (Pin 3) - 模式选择
//!
//! 运行方式:
//! ```bash
//! RUST_LOG=debug cargo run --example jtag_scan_chains
//! ```

use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_tools::{jtag::FtdiJtag, list_all_device, mpsse::FtdiMpsse};

/// 主函数 - JTAG 链扫描程序入口
///
/// 此函数实现了基本的 JTAG 链扫描功能:
/// 1. 初始化 FTDI 设备和 JTAG 接口
/// 2. 执行 JTAG 链扫描操作
/// 3. 显示扫描结果和耗时
///
/// 返回值: anyhow::Result<()> - 成功或错误信息
fn main() -> anyhow::Result<()> {
    // 初始化日志系统以显示调试信息
    env_logger::init();
    
    // 记录扫描开始时间，用于性能测量
    let now = Instant::now();
    
    // 扫描系统中所有可用的 FTDI 设备
    let devices = list_all_device();
    // 验证至少找到一个 FTDI 设备
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    
    // 打开第一个可用的 FTDI 设备的第一个接口
    // 初始化 MPSSE 模式以支持 JTAG 通信
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    // 将 MPSSE 控制器包装在线程安全的互斥锁中以支持多线程访问
    let mtx = Arc::new(Mutex::new(mpsse));
    
    // 创建 JTAG 控制器实例
    // 默认使用标准的 FTDI JTAG 引脚配置 (TCK=AD0, TDI=AD1, TDO=AD2, TMS=AD3)
    let mut jtag = FtdiJtag::new(mtx)?;
    
    // 执行 JTAG 链扫描操作
    // scan_with(true) 表示在 TDI 线上发送逻辑 1，这会触发设备返回其 IDCODE
    let ids = jtag.scan_with(true)?;
    
    // 以十六进制格式显示扫描结果
    // 每个 ID 都是 32 位的设备标识符，包含厂商、设备类型等信息
    println!("Scan Result:{ids:#x?}");
    
    // 输出扫描操作的总耗时
    println!("Finish Scan Using {:?}", now.elapsed());
    Ok(())
}
