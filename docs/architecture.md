# 目录、接口与构建

用途：说明开发入口、接口约定，以及新增应用和板卡的方法。

## 目录

```text
app/<应用>/                 app_main()、硬件绑定、车辆参数、应用 CMake
app/examples/<示例>/        rgb、rc、imu、buzzer、motor、referee、chassis、gimbal、shooter
lib/bsp/                   无 HAL / FlashDB 依赖的公共声明
lib/rgb/                   RGB/HSV 与周期变色
lib/math/unit.hpp          SI 单位换算常量
boards/<板卡>/              CubeMX 工程与生成 CMake
boards/<板卡>/bsp/          外设实现、入口、手写 CMake、工具链适配
  storage/storage.cpp      Storage 封装
  storage/flashdb/          FlashDB、FAL、配置与 Flash 实现
tools/                     presets、批量构建与格式化
tests/                     真实实现配合模拟硬件的主机测试
```

整车应用为 hero、infantry、sentry；engineer 保留已有配置，尚无完整应用入口。shooter 示例使用步兵 17 mm 双摩擦轮配置。

## 启动与句柄

CubeMX main 在全部 MX 初始化后调用：

```cpp
bsp_init();
app_main();
```

板卡 `bsp.hpp` 条件包含 C++ 声明，并用 C 链接声明这两个入口，生成的 C main 可直接包含它。BSP 启动计时、PWM、CAN、UART；USB 与 SPI 基础配置由 CubeMX 完成。

```cpp
auto can = bsp.can(1);
Can another_can(1);
auto output = bsp.pwm(4, 3);
Pwm another_output(4, 3);
Timer timer;
auto another_timer = bsp.timer();
```

Bsp 只构造句柄，构造函数保存编号并绑定板卡资源。构造不访问硬件，可在全局创建；同通道句柄共享缓冲、回调和 metrics。Timer 各自保存时间基准，首次计时建立基准。

编号允许不连续；Can、Spi 必须显式指定硬件，不提供默认构造。不存在的编号得到无效句柄，各操作静默返回且不操作硬件。外设操作不返回成败、不检查参数合法性，参数范围由调用方保证，调试看 `monitor`；实际收发在硬件初始化后执行。

Timer 提供 get_dt、update_dt、poll_timeout、reset、delay，时间为秒，默认 dt 限制为 0.0001–10 秒。底层采用 32 位周期计数；168 MHz 下观测间隔应小于约 25.56 秒，以保留完整时间信息。

## 应用绑定硬件

业务库接收句柄，app 指定通道。通信配置在对象构造时确定，注册回调后保持稳定。

```cpp
M3508 motor({.bus = {.can = Can(1), .rx_id = 0x201}});
FSi6X fsi6x({.uart = Uart(3)});
VT13 vt13({.uart = Uart(6)});
Aim aim({.cdc = bsp.cdc(1)});
Nav nav({.cdc = bsp.cdc(1)});
Buzzer buzzer(Pwm(4, 3));
Rgb rgb({.red = bsp.pwm(5, 3), .green = bsp.pwm(5, 2), .blue = bsp.pwm(5, 1)});
Imu imu({.hardware = {.accel = Spi(1, 1), .gyro = Spi(1, 2), .heater = Pwm(10, 1)}});
Storage storage("hero");
```

电机各层构造函数只接收并传递唯一的 config。公共配置分为 bus（CAN 通道、命令与反馈 ID、超时）、drive（kt、电流与转矩量程、减速比）和 install（零点偏移、方向）；config() 只读访问实际配置，校准使用 set_offset。型号物理参数使用公开的 static constexpr 独立常量，不另设全局型号配置包装。速度和位置闭环由底盘、云台、发射机构持有，Motor 只提供电流和力矩设置。DmMotor 和 Vesc 的专用 Config 内含 motor 公共配置及专用参数，DM 型号配置额外提供使能帧发送频率。

DJI 最终型号注册 CAN 回调，私有 can_callback 先校验报文并解析型号特有字段，再直接调用 Motor::can_callback 完成方向修正、多圈累计和在线计时。其他协议子类直接注册并解析 raw。update 每周期先判断反馈超时，再计算闭环；set_current、set_torque 在赋值时处理非有限值、限幅和单位换算。DJI 同组命令由 app 创建零初始化组帧，各电机通过 write_can_frame 按控制帧 ID 和反馈 ID 写入对应槽位，再由 app 统一发送。DM 电机按配置频率定时发送使能帧，该周期不发送 MIT 帧；VESC 不要求在线反馈，只支持开环速度发送。型号物理参数使用公开的 static constexpr 独立常量。

Imu 使用 hardware / adjust / control 三组配置，组合独立 Bmi088 和 QuaternionEkf 实例。构造只保存配置，首次 update 尝试初始化；失败后由后续轮询重试。温控 PID 借用内部配置，Imu 禁止复制和移动。校准由 start_calibration 发起、update 推进；转换约定和调参见 [IMU](imu.md)。

Rgb 只提供 set_rgb、set_hsv；周期变色由各车 `misc.cpp` 的 `misc_loop()` 执行。

Storage 的 read/write 保存 float：缺失键读零，写入不返回结果。所有句柄共享唯一的静态 Context，构造只绑定指针，不分配堆内存或访问 Flash；首次读写时初始化，成功后复用。首次成功初始化所用的名称决定应用标记，后续构造不切换应用或重置分区；名称字符串保持有效。对象由 app 持有，业务模块接收引用。分区仍是 768 KiB 偏移、256 KiB 长度；首次初始化发现应用标记缺失时按原规则重置配置。

## 收发与指标

```cpp
can.add_rx_callback([this](const CanFrame& frame) {
    handle_frame(frame);
});
can.send({.id = 0x123, .size = 2, .data = {1, 2}});
```

CanFrame 为经典 CAN 报文，array 容量 8 字节，size 表示实际长度；array 自身无堆分配。收发头与配置优先指定成员初始化。

- 每通道 vector 保存 std::function，支持 `[this]`，每帧在中断内依序通知全部订阅者。CAN 使用 FIFO 中断，UART 使用空闲/收满通知。
- 启动前后均可在普通上下文注册；注册时保存、关闭、恢复 PRIMASK。回调中保持短小、非阻塞，订阅列表保持稳定，捕获对象地址与生命周期保持有效。数据视图仅在回调期间有效。
- vector/std::function 可能在注册时分配内存；当前禁用异常，内存不足不保证返回 false。
- CAN 直接调用一次 HAL_CAN_AddTxMessage，邮箱满时该帧丢弃。UI 队列每次发送后直接出队。
- UART/CDC 接收 Bytes，发送先复制到 BSP 缓冲；忙时丢弃，size 不得超过缓冲容量。UART RX 128 字节、TX 256 字节；CDC TX 2048 字节，printf 最多 127 字节。
- 调试器可查看 `can.state_->metrics`，包含最近回调总耗时 last_callback_us、平滑 cpu_usage（0–1）、bus_off_count；UART 使用 error_count。统计由通道共享，独立查询 API 已移除。
- 累计回调耗时至少 100 ms 后，以实际经过时间计算占用率；平滑权重为 elapsed_ms / (400 + elapsed_ms)。仅在回调时更新，停收保留结果，恢复后将空闲时间计入区间。耗时包含全部订阅者分发，可能包含中断抢占；用于调试估算。

SPI 使用 `(总线, 片选)`。`transfer(tx_buf, rx_buf, size)` 是同步完整事务，自动拉低并恢复片选，失败也释放。普通上下文调用，BSP 使用 `HAL_MAX_DELAY` 等待，传输期间中断保持可用。Bmi088 直接持有 app 传入的两个 Spi；驱动缓存和 EKF 状态均按实例保存，无全局绑定。

## 当前板卡资源

| 资源 | 编号或映射 |
| --- | --- |
| CAN | 1、2 |
| UART | 1：中断接收；3：DMA、仅接收；6：DMA、双向 |
| CDC | 1 |
| SPI | (1,1)：BMI088 加速度计；(1,2)：陀螺仪 |
| PWM | TIM1 CH1–4、TIM4 CH3、TIM5 CH1–3、TIM8 CH1–3、TIM10 CH1 |
| RGB 接线 | app 绑定 TIM5 CH3/CH2/CH1 |

PWM 频率依据实际 APB 时钟计算，同一定时器通道共享频率，调整频率保留各占空比。占空比限制 [0,1]；ARR 保留可表示的 100% 占空比比较值。

## 构建与扩展

根 CMake 从 preset 读取 RMPP_APP，include 应用 CMake 获取 RMPP_BOARD，在 project(rmpp) 前设置工具链，再组合 lib、BSP、app。app CMake 只声明板卡；lib 为 STATIC，app/BSP 为 OBJECT。BSP 用独立二进制目录接入 CubeMX 默认工程，继续使用其打印和编译参数。

根目录保留 CMake 3.22 最低版本声明；缺少 RMPP_APP 直接报错。Preset 的 cacheVariables 保存在构建目录；更换板卡应使用新的构建目录。
工具链文件在识别编译器前加载，修正搬迁后的链接脚本和 map 路径；BSP CMake 在此后创建编译目标。DSP 依赖由板卡接入。

新增应用：复制 app 的 CMake、实现 app_main、绑定外设，然后运行 `python tools/update_presets.py`。每个应用只生成默认 Debug 配置预设，名称不带构建类型后缀；Release 按需在 CLion 中复制配置后修改。产物为 `build/<preset>/rmpp.elf` 和 `rmpp.map`。
新增板卡：放入 CubeMX 工程，提供 BSP 工具链和 `lib/bsp` 接口实现，定义硬件映射；C main 和 USB 桥接位于 USER CODE 区。

`CMAKE_CURRENT_SOURCE_DIR` 是当前 add_subdirectory 的源目录，include 不改变它；根目录因此直接用 RMPP_APP 拼接应用路径。手写源码用 `GLOB_RECURSE CONFIGURE_DEPENDS "*.*"` 扫描。

## 格式化

CLion 启用项目 `.clang-format`。使用同一个 clang-format 可执行文件运行：

```sh
python tools/format_code.py --clang-format <CLion使用的clang-format路径>
python tools/format_code.py --clang-format <相同路径> --check
```

当前验证版本为 CLion 随附 clang-format 22。LLVM 基础、四空格、同行大括号、无列宽限制；普通函数展开，函数定义之间自动留空行，内部逻辑块由编写者分组。
