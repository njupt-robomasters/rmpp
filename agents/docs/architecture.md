# 目录、接口与构建

用途：说明开发入口、接口约定，以及新增应用和板卡的方法。

## 目录

```text
app/<应用>/                 app_main()、硬件绑定、车辆参数、应用 CMake
app/examples/<示例>/        empty、rgb、rc、imu、attitude、melody、cdc、kv_store、motor、referee、chassis、gimbal、shooter
lib/bsp/                   无 HAL 依赖的公共外设声明（Can、Uart、Cdc、Rgb、Buzzer、Imu、KvStore、Timer、Monitor）
lib/attitude/              姿态解算：抽象父类 Attitude 与算法子类 QuaternionEkf
lib/math/                  单位、角度、低通滤波、Eigen 类型与姿态转换
lib/power/                 超级电容 SuperCap（G474SuperCap CAN 协议，app 持有并传给底盘）、功率限制 PowerLimiter（底盘按值持有，缓冲能量功率预算兼作超电目标）
lib/melody/                  Melody 与曲目
lib/third_party/           纯软件第三方库：Eigen
third_party/               含硬件移植层、由板卡 BSP 编译的第三方库：CherryUSB v1.6.1、FlashDB 2.2.0
boards/<板卡>/              CubeMX 工程与生成 CMake
boards/<板卡>/bsp/          外设实现、入口、手写 CMake、工具链适配、usb_config.h
  include/board/           板卡专用接口（如 mc02 的 power.hpp）
  flashdb/                 FlashDB 配置 fdb_cfg.h、fal_cfg.h 与 Flash 驱动
tools/                     presets、批量构建与格式化
```

整车应用为 infantry、sentry，均使用 C 板。shooter 示例使用步兵 17 mm 双摩擦轮配置；kv_store 示例两板通用，默认 C 板，`-DRMPP_BOARD=mc02` 配置到独立构建目录即为 mc02。

## 启动与句柄

CubeMX main 在全部 MX 初始化后，在 USER CODE 2 中调用：

```c
extern void bsp_init();
extern void app_main();

bsp_init();
app_main();
```

`bsp_init()` 启动计时、CAN、UART 接收、USB、PWM 通道等，各板在 `bsp/bsp.cpp` 中列出。

```cpp
Can can(1);
Uart uart(6);
Cdc cdc(1);
Rgb rgb;
Imu imu;
Timer timer;
```

`Can`、`Uart`、`Cdc` 只保存编号，构造不访问硬件，可在全局创建；同编号句柄共享 BSP 内部的缓冲、回调和 monitor。编号允许不连续，不存在的编号各操作静默返回。`Rgb`、`Buzzer` 是板载唯一外设，无需参数。细节见 [BSP](bsp.md)。

Timer 提供 `elapsed`、`restart`、`delay`，时间为秒，可在构造时限定 dt 范围；底层为 64 位 CPU 周期计数，不存在溢出间隔。

## 应用绑定硬件

业务库接收句柄，app 指定通道。通信配置在对象构造时确定，注册回调后保持稳定。

```cpp
M3508 motor({.bus = {.can = Can(1), .rx_id = 0x201}});
FSi6X fsi6x({.uart = Uart(3)});
VT13 vt13({.uart = Uart(6)});
Aim aim({.cdc = Cdc(1)});
Rgb rgb;
Melody melody;
Imu imu({.gyro_offset = Vector3f(0.003f, -0.004f, -0.002f), .gravity = 9.80f});
QuaternionEkf attitude(imu);
KvStore kv_store("infantry");
```

电机各层构造函数只接收并传递唯一的 config。公共配置分为 bus（CAN 通道、命令与反馈 ID、超时）、drive（kt、电流与转矩量程、减速比）和 install（零点偏移、方向）；`Motor::config` 公开，供功率限制、云台零点存储和发射卡弹判断读取 drive 与 install，校准使用 set_offset。型号物理参数使用公开的 static constexpr 独立常量，不另设全局型号配置包装。速度和位置闭环由底盘、云台、发射机构持有，Motor 只提供电流和力矩设置。DmMotor 和 Vesc 的专用 Config 内含 motor 公共配置及专用参数，DM 型号配置额外提供使能帧发送频率。

DJI 最终型号注册 CAN 回调，私有 can_callback 先校验报文并解析型号特有字段，再直接调用 Motor::can_callback 完成方向修正、多圈累计和在线计时。其他协议子类直接注册并解析 raw。update 每周期先判断反馈超时，再计算闭环；set_current、set_torque 在赋值时处理非有限值、限幅和单位换算。DJI 同组命令由 app 创建零初始化组帧，各电机通过 write_can_frame 按控制帧 ID 和反馈 ID 写入对应槽位，再由 app 统一发送。DM 电机按配置频率定时发送使能帧，该周期不发送 MIT 帧；VESC 不要求在线反馈，只支持开环速度发送。

BSP `Imu` 负责 BMI088 读取、恒温加热、零偏校准和安装方向修正，输出机体系量，可单独使用；`Attitude` 是接收 `Imu&` 的抽象父类，输出 q、欧拉角和累计 yaw，算法由子类（如 `QuaternionEkf`）实现。云台持有 `Attitude& attitude`。转换约定和调参见 [IMU](imu.md)。

`Rgb` 提供 `set(red, green, blue)` 和 `set_hsv`，范围 [0, 1]；周期变色由各车 `misc.cpp` 的 `misc_loop()` 执行。`Melody` 在板载 `Buzzer` 上播放曲目，见 [蜂鸣器曲目](music.md)。

KvStore 的 read/write 保存 float：缺失键读零，写入不返回结果。所有实例共享同一数据库，构造不访问 Flash、可早于 `bsp_init()`，首次读写时初始化，首个构造实例的名称决定应用标记，更换名称时格式化；名称字符串保持有效。C 板分区为内部 Flash 768 KiB 偏移、256 KiB 长度；mc02 为板载 W25Q64 从 0 起 1 MiB。

## 收发

```cpp
can.add_rx_callback([this](const CanFrame& frame) {
    handle_frame(frame);
});
can.send({.id = 0x123, .size = 2, .data = {1, 2}});
```

CanFrame 为经典 CAN 报文，数据容量 8 字节，size 表示实际长度。

- 每通道 vector 保存 std::function，支持 `[this]`，每帧在中断内依序通知全部订阅者。
- 启动前后均可在普通上下文注册；回调保持短小、非阻塞，数据视图仅在回调期间有效。vector/std::function 可能在注册时分配内存。
- C 板 CAN 直接调用一次 `HAL_CAN_AddTxMessage`，邮箱满时丢弃；mc02 写入 32 级 TX FIFO。
- UART/CDC 发送先复制到 BSP 缓冲，忙时丢弃；size 不得超过缓冲容量（UART 512、CDC 2048 字节）。
- 调试器可查看 BSP 文件内的 `contexts[i].monitor`（freq、cpu 及低通值）、CAN 的 `bus_off_count`、UART 的 `error_count`。

## 当前板卡资源

| 资源 | C 板（STM32F407） | mc02（STM32H723） |
| --- | --- | --- |
| CAN | 1、2 | 1、2、3（FDCAN 经典帧） |
| UART | 1、3（DBUS，只收）、6 | 1、5（DBUS，只收）、7、10 |
| CDC | 1 | 1、2 |
| Rgb | TIM5 CH3/CH2/CH1 | WS2812，SPI6 |
| Buzzer | TIM4 CH3 | TIM12 CH2 |
| Imu | SPI1，加热 TIM10 CH1 | SPI2，加热 TIM3 CH4 |
| KvStore | FlashDB，内部 Flash | FlashDB，W25Q64 |
| 板卡专用 | 无 | `board/power.hpp`：OUT1、OUT2、OUT_5V |

## 构建与扩展

根 CMake 从 preset 读取 RMPP_APP，include 应用 `board.cmake` 获取 RMPP_BOARD，在 project(rmpp) 前设置工具链，再组合 lib、BSP、app。细节见 [构建](build.md)。

新增应用：复制 app 的 CMake 和 `board.cmake`、实现 app_main、绑定外设，然后运行 `python tools/update_presets.py`。产物为 `build/<preset>/rmpp.elf` 和 `rmpp.map`。
新增板卡：放入 CubeMX 工程，提供 `bsp/toolchain.cmake`、`bsp/CMakeLists.txt` 和 `lib/bsp` 接口实现；main.c 的入口调用位于 USER CODE 区。

## 格式化

CLion 启用项目 `.clang-format`。使用同一个 clang-format 可执行文件运行：

```sh
python tools/format_code.py --clang-format <CLion使用的clang-format路径>
python tools/format_code.py --clang-format <相同路径> --check
```

CLion 2026.2.2 随附的 clang-format 位于 `plugins/clion-radler/DotFiles/windows-x64/clang-format.exe`。LLVM 基础、四空格、同行大括号、无列宽限制。
