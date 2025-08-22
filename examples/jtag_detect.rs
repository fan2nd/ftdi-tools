//! JTAG 设备自动检测示例
//!
//! 此示例演示如何使用 FTDI 芯片自动检测 JTAG 引脚配置并识别连接的设备。
//! JTAG (Joint Test Action Group) 是一种广泛使用的调试和测试接口。
//!
//! JTAG 信号线说明:
//! - TCK (Test Clock): 测试时钟信号
//! - TMS (Test Mode Select): 测试模式选择
//! - TDI (Test Data In): 测试数据输入
//! - TDO (Test Data Out): 测试数据输出
//!
//! 检测原理:
//! 1. 首先遍历所有可能的 TCK/TMS 组合查找 TDO
//! 2. 然后在找到的 TDO 组合中找 TDI
//! 3. 通过比较不同 TDI 输入下的扫描结果来确认正确的引脚配置
//!
//! 运行方式:
//! ```bash
//! RUST_LOG=info cargo run --example jtag_detect
//! ```

use std::time::Instant;

use ftdi_tools::{
    jtag::{JtagDetectTdi, JtagDetectTdo},
    list_all_device,
    mpsse::FtdiMpsse,
};
use itertools::Itertools;

/// 主函数 - JTAG 设备检测程序入口
///
/// 此函数实现了分两阶段的 JTAG 检测算法:
/// 阶段1: TDO 检测 - 遍历 TCK/TMS 组合查找可能的 TDO 引脚
/// 阶段2: TDI 检测 - 在找到的 TDO 组合中查找正确的 TDI 引脚
///
/// 返回值: anyhow::Result<()> - 成功或错误信息
fn main() -> anyhow::Result<()> {
    // 初始化日志系统以输出调试信息
    env_logger::init();

    // 记录检测开始时间，用于计算整个检测过程的耗时
    let now = Instant::now();

    // 扫描系统中所有可用的 FTDI 设备
    let devices = list_all_device();
    // 确保至少找到一个可用的 FTDI 设备
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    // 打开第一个 FTDI 设备的第一个接口，初始化 MPSSE 模式
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;

    // 创建存储没有找到 TDI 的引脚组合的向量
    // 这些组合将在第二阶段用于 TDI 检测
    let mut notdi: Vec<_> = Vec::new();

    // =========================
    // 阶段1: TDO 检测阶段
    // =========================
    // 遍历所有可能的 TCK 和 TMS 引脚组合 (8 个引脚中选择 2 个的排列)
    for couple in (0..8).permutations(2) {
        let tck = couple[0]; // TCK (测试时钟) 引脚编号
        let tms = couple[1]; // TMS (测试模式选择) 引脚编号

        // 创建 TDO 检测器实例
        // 这个检测器会尝试在指定的 TCK/TMS 组合下查找 TDO 信号
        let jtag = JtagDetectTdo::new(&mpsse, tck, tms)?;

        // 注意: 在实际应用中，这里可以添加控制电平转换芯片的代码
        // 例如设置 TCK 和 TMS 为输出，其他引脚为输入

        // 执行 JTAG 链扫描，查找可能的 TDO 引脚
        jtag.scan()?
            .into_iter()
            .for_each(|tdo| notdi.push((tck, tms, tdo))); // 将找到的组合保存起来
    }

    // =========================
    // 阶段2: TDI 检测阶段
    // =========================
    // 遍历第一阶段找到的所有可能的 TCK/TMS/TDO 组合
    for (tck, tms, tdo) in notdi {
        // 注意: 在实际应用中，这里可以添加控制电平转换芯片的代码
        // 例如设置 TDO 为输入，其他引脚为输出

        // 遍历所有可能的 TDI 引脚 (0-7)
        for tdi in 0..8 {
            // 跳过与已经使用的引脚冲突的情况
            if tdi == tck || tdi == tms || tdi == tdo {
                continue;
            }

            // 创建 TDI 检测器实例
            // 这个检测器会测试完整的 JTAG 引脚配置
            let jtag = JtagDetectTdi::new(&mpsse, tck, tdi, tdo, tms)?;

            // 执行两次不同的扫描测试
            let ids_scan1 = jtag.scan_with(true)?; // 使用逻辑 1 进行扫描
            let ids_scan0 = jtag.scan_with(false)?; // 使用逻辑 0 进行扫描

            // JTAG 检测的关键算法:
            // 当 TDI 输入为 1 时，正常的 JTAG 链会比 TDI 输入为 0 时多返回 32 个单元
            // 这是因为 JTAG 设备在无效数据时会进入 bypass 模式
            if ids_scan0.len() - ids_scan1.len() == 32 {
                // 找到正确的 JTAG 引脚配置!
                println!("!!!!!! Pins:tck[{tck}],tdi[{tdi}],tdo[{tdo}],tms[{tms}]");
                // 输出检测到的 JTAG 设备 ID 列表
                println!("!!!!!! Found Devices:{ids_scan1:x?}");
            }
        }
    }

    // 输出整个检测过程的耗时
    println!("Finish Detect Using {:?}", now.elapsed());
    Ok(())
}
