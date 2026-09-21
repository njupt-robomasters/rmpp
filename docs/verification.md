# 验证与人工审计

用途：记录本次多板卡重构的软件验证，并给出人工审计和板上验证入口。

## 已完成

- 重构前快照：`46f2112`，包含开始实施时的源码、文档及既有删除记录。
- Arm GNU 15.3.1、CMake/Ninja：三个整车与九个示例的 Debug/Release，共 24 组构建通过。
- MinGW GNU 15.2：直接编译 BSP 实现的主机测试通过，测试代码在 `tests/`。
- 检查全部 24 个 ELF：main、bsp_init、app_main 各一个，CAN/UART/USB 接收回调均为实际链接的强符号。
- 对照 143 个生成及厂商文件：仅 main.c、usbd_cdc_if.c 的 USER CODE 接入发生变化，其余内容随目录原样搬迁。
- CubeMX 根 CMake 仅移除原手写 `add_subdirectory(rmpp)`，默认生成部分保留。
- 三个整车共 18 个配置头文件的数值字面量核对一致；新增 PWM 句柄选择单独核对。
- FlashDB 分区配置与快照一致；现有协议字段、生成 MAVLink、数据键和校准含义保留。

主机测试覆盖：全局句柄与启动前注册、无效/稀疏 ID、共享句柄、捕获 this、多订阅者顺序、通道隔离、连续多帧、CAN 单次提交及邮箱满、嵌套中断屏蔽恢复、UART 空闲/收满/错误后重新接收、UART/CDC 发送缓冲区所有权与忙状态、CDC 截断、PWM APB 时钟/占空比/共享频率、LED 映射、独立 Timer/回绕/复位/限幅，以及 C 文件包含 bsp.hpp 并调用 C++ 实现。

## 资源占用

| 应用 | Debug RAM / Flash（B） | Release RAM / Flash（B） |
| --- | ---: | ---: |
| hero | 36336 / 117576 | 36264 / 125812 |
| infantry | 35680 / 116088 | 35608 / 122660 |
| sentry | 36160 / 118456 | 36088 / 129012 |

CCMRAM 均为零。hero Debug 相比快照基线增加 RAM 3272 B、Flash 8584 B；新增固定发送缓冲区、通道状态和外设实现占用空间。
链接报告不包含运行时 vector/std::function 实际堆分配量及最大栈深度。

## 复现

工具已加入 PATH 时，在根目录执行：

```sh
python tools/update_presets.py
python tools/check_builds.py
cmake -S tests -B build/host -G Ninja
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

主机测试使用本机 C/C++ 编译器；交叉工具链由固件 app 选择，两者使用不同构建目录。
Windows 可通过 `-DCMAKE_C_COMPILER=...`、`-DCMAKE_CXX_COMPILER=...` 指定 MinGW。
批量构建脚本支持 `--cmake`、`--ninja` 指定路径，完整日志位于 `build/checks/`。

## 建议审计顺序

1. `lib/board/`：接口、空句柄、CanFrame、Bytes、Timer。
2. `boards/stm32f407_igh6/bsp/`：查找表、bsp_init、发送与回调、PWM 和计时实现。
3. `app/hero/app.cpp` 及应用配置：入口、控制周期、PWM 句柄和发送顺序。
4. 根、app、lib、BSP 的 CMake，以及 `tools/update_presets.py`。
5. 业务代码的命名和调用迁移；UI 队列现仅在 CAN 提交成功后出队。

## 板上待验证与既有问题

主机模拟没有测量真实中断耗时、注册临界区延迟、总线负载、USB 枚举/断连、PWM 波形和机械行为。需要审计后在板上验证。
CAN 成功仅表示已提交邮箱；整车周期发送没有新增队列，忙时本次报文返回失败，下一控制周期再发送新值。
UART 当前在通知回调后重新接收，需按真实数据速率检查回调耗时与接收间隙。
接收回调避免阻塞和动态分配，捕获对象保持存活；运行时注册会在关中断期间扩容，需测量实际最长延迟。

Release 仍有第三方 QuaternionEKF 类型重解释/未初始化分析和 ui_interface 数组边界警告，与历史记录一致，未为通过构建屏蔽它们。
双舵轮功率公式等既有算法问题见 `unit-migration.md`；本次未调参、烧录或上车测试。
