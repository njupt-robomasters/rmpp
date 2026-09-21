# 目录、接口与构建

用途：说明日常开发入口，以及增加应用和板卡的方法。

## 目录职责

```text
app/<应用>/                业务入口 app_main()、配置、应用 CMake
app/examples/<示例>/       led、rc、imu、buzzer、motor、referee、chassis、gimbal、shooter
lib/                       公共算法和业务库
lib/board/                 无 HAL 依赖的外设声明
boards/stm32f407_igh6/      原 CubeMX 工程及生成 CMake
boards/<板卡>/bsp/         外设实现、bsp.hpp、手写 CMake、工具链适配
tools/                     预设生成与批量构建
tests/                     用模拟 HAL 执行真实 BSP 实现
```

现有整车应用为 hero、infantry、sentry；engineer 保留已有代码，尚无构建入口。
各示例独立持有必要配置；shooter 使用步兵 17 mm 双摩擦轮配置。

## 入口与接口

CubeMX `main.c` 在全部 `MX_*_Init()` 后执行：

```c
bsp_init();
app_main();
```

两函数在板卡 `bsp.hpp` 中声明，通过条件编译兼容 C，通过 `extern "C"` 保持链接名称。
BSP 启动计时、PWM、CAN 和 UART 接收；USB 沿用 CubeMX 初始化。
应用只编写业务初始化和循环。

```cpp
using rmpp::board;

auto can = board.can(1);
auto uart = board.uart(3);
auto usb = board.cdc(1);
auto led = board.led(1);
auto output = board.pwm(4, 3);
auto timer = board.timer();

can.add_rx_callback([this](const rmpp::CanFrame& frame) {
    receive(frame);
});
can.send({.id = 0x123, .data = {1, 2}, .size = 2});

output.set_frequency(2000);
output.set_duty(0.5f);
output.set_duty(0);
```

Board 只查找资源；各 class 实现行为。句柄可以全局构造和按值复制，同一通道共享状态。
函数内静态通道状态采用常量初始化，构造不调用 HAL。Timer 各自保存计时基准，首次计时操作建立基准。

整数编号允许不连续。无效编号返回空句柄，`bool(handle)` 为 false，操作返回 false、数值查询返回零，不访问硬件。
bool 只表示资源存在；硬件操作在初始化后执行。

`CanFrame` 包含 ID、8 字节 array、实际 size、extended、remote。array 自身无堆分配，容量与报文实际长度分别保存。
当前接口支持经典 CAN；增加 CAN FD 数据长度时再扩展报文能力。

## 接收与发送

- `add_rx_callback()` 只传一个 callable。std::function 支持捕获 lambda，每个通道的 vector 按注册顺序保存全部订阅者。
- 每帧在中断内通知所有订阅者，业务自行判断 ID。CAN 使用 FIFO 中断；UART 使用空闲及缓冲区收满事件；无需接收 poll。
- 可在启动前后注册。普通上下文注册时保存 PRIMASK、屏蔽中断、追加回调，再恢复原中断状态；回调中不修改订阅列表。
- 回调保持短小、非阻塞；捕获对象持续存活且地址稳定，收到的数据视图只在回调期间有效。没有取消订阅接口。
- vector 与 std::function 可能在注册时分配内存；当前关闭异常，内存耗尽不承诺返回 false。
- CAN 发送直接调用一次 `HAL_CAN_AddTxMessage()`，返回是否成功提交；不等待邮箱。业务决定下次是否重试，UI 已有队列只在提交成功后出队。
- UART/CDC 发送参数为 `Bytes{data, size}`，先复制到 BSP 固定发送缓冲区，忙或超长返回 false；成功表示提交，不代表对端收到。
- UART TX 容量 256 字节、RX 容量 128 字节；CDC TX 容量 2048 字节。容量在 BSP 实现中定义。
- CDC `printf()` 最多发送 127 字节，较长输出截断；保留每通道负载及错误计数查询。

## 当前板卡资源

| 资源 | 可用编号或通道 |
| --- | --- |
| CAN | 1、2 |
| UART | 1：中断接收；3：DMA、仅接收；6：DMA、双向 |
| CDC | 1 |
| LED | 1：RGB，TIM5 CH3/CH2/CH1 |
| PWM | TIM1 CH1–4、TIM4 CH3、TIM5 CH1–3、TIM8 CH1–3、TIM10 CH1 |
| Timer | DWT 周期计数器作为板卡内部实现 |

PWM 频率按实际 APB 时钟及倍频计算，同一定时器的通道共享频率，调整频率保持各占空比。
占空比限制在 [0,1]；非法或非有限频率失败。使用最多 65535 个计数，保留完整 100% 输出对应的比较值。

业务蜂鸣器接收一个 Pwm：`Buzzer buzzer(board.pwm(4, 3))`。
IMU 构造的第三个参数是加热 Pwm：`Imu imu(dir, calib, board.pwm(10, 1))`。
波特率、引脚、滤波和启动配置仍由 CubeMX/BSP 定义。

Timer 方法：get_dt、update_dt、poll_timeout、reset、delay。时间为秒，默认区间 0.0001–10 秒。
32 位周期差可跨一次回绕；168 MHz 下两次观测应相隔小于约 25.56 秒，无法据此恢复遗漏的完整回绕次数。

BMI088 使用板卡的 SPI、片选、延时 C 函数。FlashDB 平台实现及分区在 BSP，分区仍为 768 KiB 偏移、256 KiB 长度，保留数据键与校准含义。

## CMake 与新增目标

根 CMake 先 include 所选 app 的 CMake，确定板卡及工具链，再 project()、调用 app 定义的 rmpp_add_app()。
每个 app 的单个 CMake 文件设置默认 RMPP_BOARD，并组合 lib、BSP、应用源码。
BSP 用 add_subdirectory 添加上一级生成工程，再把自己的对象和业务库链接到 CubeMX 创建的 rmpp 固件。

lib 为 STATIC，app/BSP 为 OBJECT，保证 HAL 弱回调的覆盖实现进入固件。
stm32cubemx 提供 HAL 头文件和宏；生成工具链提供 CPU/FPU/ABI、Debug/Release 参数。
手写工具链适配器修正链接脚本及 map 路径。DSP 和 FlashDB 配置依赖由板卡接入处提供。

`CACHE STRING` 将选择保存到构建目录，可用 -D 或 Preset 覆盖。切换板卡使用新构建目录。
`CMAKE_CURRENT_SOURCE_DIR` 是当前 add_subdirectory 的源目录；include 不改变它，因此 app 用 CMAKE_CURRENT_LIST_DIR 保存自己的位置。

新增应用：复制一个 app 的 CMake，添加 app_main()，调整默认板卡，执行 `python tools/update_presets.py`。
脚本递归发现应用，为每个应用生成 Debug/Release；一次 configure 只构建一个应用与板卡组合。
产物位于 `build/<preset>/rmpp.elf` 和 `rmpp.map`。

新增板卡：把 CubeMX 工程放到 boards 下，复制 BSP 接入结构，实现 lib/board 声明，配置资源编号与平台函数。
生成代码接入写在 USER CODE 区域；保留生成 CMake 的打印和参数。用 `--verbose` 查看编译命令。
