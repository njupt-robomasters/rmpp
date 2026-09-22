# 验证与人工审计

用途：说明当前实现的验证结果、复现方法及板上待检查内容。

## 软件验证

- 基线为 `fefa258`；更早的目录重构前快照为 `46f2112`。
- Arm GNU 15.3.1、CMake 4.3.1/Ninja：三个整车与九个示例的 Debug/Release，共 24 组构建通过。
- MinGW GNU 15.2：BSP 与 Storage 两组主机测试通过。编译真实 BSP、SPI 适配和 FlashDB/FAL，仅模拟硬件。
- 24 个 ELF 均包含唯一的 main、bsp_init、app_main，以及实际链接的 CAN/UART/CDC 接收回调强符号。
- 对照基线，203 个生成或第三方文件内容一致（忽略行结束符差异），包含移入 BSP 的 FlashDB/FAL；CubeMX 生成工程和配置保留。
- 三个整车共 18 个配置头文件的控制数值核对一致；原 CAN 通道选择逐项对应，PWM/SPI/串口绑定按 app 配置核对。单位换算常量保留。
- 146 个手写 C/C++ 文件使用 CLion 随附 clang-format 22 格式化，二次检查通过。根配置同时约束函数间空行。
- 根 CMake 缺少 RMPP_APP 时明确失败，提示通过 preset 选择应用。

主机测试覆盖：全局构造、提前注册、无效/稀疏编号、直接构造与共享句柄、多个捕获回调、通道隔离、FIFO 连续帧、CAN 单次提交及邮箱满、PRIMASK 恢复、UART 空闲/收满/错误重启、UART/CDC 发送缓冲和忙状态、PWM 时钟/占空比/共享频率、RGB、独立 Timer 与回绕、C 到 C++ 入口。

新增验证包括：

- 指标：每毫秒 20 μs 与成批到达都收敛到 2%，负载变化、停收恢复和毫秒计数回绕符合预期。GDB 实际检查通道 metrics，最近回调为模拟的 10 μs，另一通道的 bus_off_count 为 1。
- SPI：总线/片选映射、非法参数无引脚操作、忙状态拒绝、成功/失败均恢复片选；BMI088 单寄存器与连续读取的地址、空读字节和数据位置符合原实现。
- Storage：构造不读写 Flash；同名重新创建后数据保留；应用切换遵循原标记重置规则；原 FlashDB API 与 Storage 交叉读写相同 float blob。模拟 Flash 检查地址范围、擦除对齐和只能将位写为零的约束。

## 固件资源

| 应用 | Debug RAM / Flash（B） | Release RAM / Flash（B） |
| --- | ---: | ---: |
| hero | 35640 / 117860 | 35568 / 126812 |
| infantry | 34976 / 116468 | 34904 / 123728 |
| sentry | 35456 / 118864 | 35384 / 130200 |

CCMRAM 均为零。链接报告不包含 vector/std::function、Storage 内部状态的实际堆分配量及最大栈深度。

## 复现

工具已加入 PATH 时，在根目录执行：

```sh
python tools/update_presets.py
python tools/check_builds.py
cmake -S tests -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host
ctest --test-dir build/host --output-on-failure
python tools/format_code.py --clang-format <CLion使用的clang-format路径> --check
```

主机测试使用本机 C/C++ 编译器；固件使用 app 选择的交叉工具链。Windows 可用 -DCMAKE_C_COMPILER、-DCMAKE_CXX_COMPILER 指定 MinGW。
批量构建脚本支持 --cmake、--ninja，完整日志在 `build/checks/`。
当前格式化可执行文件为 CLion 安装目录中的 `plugins/clion-radler/DotFiles/windows-x64/clang-format.exe`；CLion 与命令行应使用同一版本。

## 人工审计入口

1. `lib/bsp/`：公共接口、无效句柄、Bsp 入口。
2. 板卡 `bsp/`：构造绑定、metrics、临界区、收发与 PWM/SPI 实现。
3. BMI088 通信适配与板卡 `bsp/storage/`：通信事务与存储。
4. app 的硬件绑定、控制周期与发送顺序；业务库中的配置传递。
5. 根、app、lib、BSP 的 CMake，以及 .clang-format。

## 板上验证与既有问题

软件验证未测量真实中断耗时、注册时关中断延迟、总线负载、USB 枚举/断连、PWM 波形和机械行为。用户审计后决定板上测试。
CAN 成功表示提交邮箱；整车周期发送在忙时返回失败，下个周期发送新值。UART 在通知订阅者后重新接收，需检查实际回调耗时与接收间隙。SPI 同步事务在普通上下文调用。
metrics 仅在收到报文时更新，停收保留最后结果；占用率表示回调分发的估算负载，可能包含抢占时间。
Storage 按现有规则切换应用时重置配置；当前每个应用持有一个实例。

Release 仍有原第三方 QuaternionEKF 类型重解释/未初始化分析、ui_interface 数组边界警告。64 位主机编译 FlashDB 时另有原实现使用 32 位地址转换的警告；固件为 32 位平台。
原 BMI088 驱动初始化持续重试、EKF 单实例，以及双舵轮/VESC 等既有问题继续保留，历史说明见 `unit-migration.md`。本轮未烧录或上车测试。
