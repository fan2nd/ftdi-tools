//! SWD (Serial Wire Debug) 读取设备 ID 示例
//!
//! 此示例演示如何使用 FTDI 芯片通过 SWD 接口连接并读取 ARM Cortex-M 处理器的 IDCODE。
//! SWD 是 ARM 开发的一种 2 线调试接口，是 JTAG 的简化版本。
//!
//! SWD 接口信号:
//! - SWCLK (Serial Wire Clock): 串行时钟信号
//! - SWDIO (Serial Wire Data I/O): 双向数据信号
//!
//! 硬件连接:
//! - SWCLK: FTDI AD0 (Pin 0) - 时钟输出
//! - SWDIO: FTDI AD1 (Pin 1) - 数据输出
//! - SWDIO_INPUT: FTDI AD2 (Pin 2) - 数据输入,需要和AD1短接
//! - VCC: 3.3V
//! - GND: 接地
//!
//! 支持的目标设备:
//! - ARM Cortex-M0/M0+/M3/M4/M7/M33/M55 等
//! - STM32, NXP LPC, Nordic nRF, Atmel SAM 等微控制器
//!
//! IDCODE 示例值:
//! - STM32G431: 0x2BA01477 (按照 RM0440-47.8.5)
//! - STM32F103: 0x1BA01477
//! - nRF52832: 0x2BA01477
//!
//! 运行方式:
//! ```bash
//! RUST_LOG=debug cargo run --example swd_read_id
//! ```

use std::sync::{Arc, Mutex};

use ftdi_tools::{
    list_all_device,
    mpsse::FtdiMpsse,
    swd::{FtdiSwd, SwdAddr},
};

/// 主函数 - SWD IDCODE 读取程序
///
/// 此函数实现了基本的 SWD 通信流程:
/// 1. 初始化 FTDI 设备和 SWD 接口
/// 2. 激活 SWD 通信协议
/// 3. 读取目标设备的 IDCODE 寄存器
/// 4. 显示设备标识信息
///
/// 返回值: anyhow::Result<()> - 操作结果
fn main() -> anyhow::Result<()> {
    // 初始化日志系统以显示 SWD 通信调试信息
    env_logger::init();

    // 获取系统中所有可用的 FTDI 设备列表
    let devices = list_all_device();
    // 验证至少存在一个 FTDI 设备可供使用
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    // 打开第一个 FTDI 设备的第一个接口
    // 初始化 MPSSE 模式以支持 SWD 通信协议
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    // 使用线程安全的互斥锁包装 MPSSE 控制器
    let mtx = Arc::new(Mutex::new(mpsse));

    // 创建 SWD (Serial Wire Debug) 接口实例
    // 这将配置 FTDI 引脚以支持 SWD 协议
    let swd = FtdiSwd::new(mtx)?;

    // 激活 SWD 接口
    // 这个操作会发送 SWD 激活序列以将目标设备从 JTAG 模式切换到 SWD 模式
    swd.enable()?;

    // 读取调试端口 (DP) 的 IDCODE 寄存器
    // SwdAddr::Dp(0) 指向 DP 的寄存器地址 0，即 IDCODE 寄存器
    // IDCODE 包含了调试端口的版本和设计信息
    let idcode = swd.read(SwdAddr::Dp(0))?;

    // 显示读取到的 IDCODE 值
    // 对于 STM32G431CBU6，这个值应该是 0x2BA01477 (根据 RM0440-47.8.5)
    // IDCODE 的结构:
    // - [31:28]: 版本号
    // - [27:12]: 部件号
    // - [11:1]:  设计商 ID (ARM = 0x23B)
    // - [0]:     固定为 1
    println!("SWD IDCODE: {idcode:#010x}");
    Ok(())
}
