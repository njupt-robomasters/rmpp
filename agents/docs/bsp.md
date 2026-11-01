# BSP 开发约定

仅在修改 `lib/bsp/**` 或 `boards/**/bsp/**` 时读取本文档

## 接口一览

| 接口 | 形式 | C 板 | mc02 |
| --- | --- | --- | --- |
| `Can(id)` | 编号句柄 | 1、2（bxCAN） | 1–3（FDCAN 经典帧） |
| `Uart(id)` | 编号句柄 | 1、3（DBUS 只收）、6 | 1、5（DBUS 只收）、7、10 |
| `Cdc(id)` | 编号句柄 | 1 | 1、2 |
| `Rgb` | 板载唯一 | TIM5 PWM | WS2812（SPI6） |
| `Buzzer` | 板载唯一 | TIM4 CH3 | TIM12 CH2 |
| `Imu` | 板载唯一 | BMI088 SPI1，加热 TIM10 CH1 | BMI088 SPI2，加热 TIM3 CH4 |
| `KvStore` | 共享数据库 | FlashDB，内部 Flash | FlashDB，W25Q64（OCTOSPI1） |
| `Timer` | 实例计时 | 64 位周期计数 | 同左 |
| `board/power.hpp` | 板卡专用 | 无（引用时编译报错） | `set_power(PowerOutput, bool)`，上电全关 |

## 构造与启动

- `Can`、`Uart`、`Cdc` 只保存编号 `id_`，构造 constexpr，可在全局创建且不访问硬件
- 运行数据放在各板外设 `.cpp` 的文件私有 static 表（`CanContext`、`UartContext`、`CdcContext`），头文件不暴露；操作时按编号查表，中断回调按 HAL 句柄查表
- 不存在的编号各操作静默返回
- `Rgb`、`Buzzer` 无状态，可任意构造；`Imu` 持有温控配置和 PID 状态，由 app 创建唯一实例
- `KvStore` 所有实例共享同一数据库，首次构造时初始化
- 硬件启动由板卡 `bsp_init()` 执行，在 CubeMX 外设初始化后、`app_main()` 前调用

## 初始化入口

- 仅供板卡入口调用的初始化函数在板卡 `bsp.cpp` 中直接 `extern` 声明
- 不为这些函数另设公共头文件
- C 与 C++ 共用入口使用 `extern "C"`
- 文件内部处理函数使用 `static`
- 文件内部函数不提供独立公开声明

## 句柄与接口

- Can、Uart、Cdc 必须显式指定编号，不提供默认空句柄
- 型号默认参数不包含硬件句柄
- 外设不提供 `operator bool()`
- 发送接口返回请求是否被底层接受
- 参数范围由调用方保证，不重复检查
- 必要的运行状态判断留在实现中：UART 上次 DMA 未完成、CDC 未枚举或上次发送未完成时返回 false
- 板卡独有的功能放在 `boards/<板卡>/bsp/include/board/*.hpp`，根 CMake 把该目录加入 `rmpp_app`；其他板卡没有该头文件，引用时编译报错
- 调试统计通过 `monitor` 查看

## 回调

- 每通道 vector 保存 `std::function`，在接收中断内依序通知；CAN 用 FIFO 中断，UART 用空闲中断，CDC 在 USB 中断内
- 注册时保存、关闭、恢复 PRIMASK；回调保持短小、非阻塞，数据视图仅在回调期间有效

## Monitor

- 共用调试统计统一为 `Monitor/monitor`，放在 `lib/bsp/monitor.hpp`
- `begin()`/`end()` 包住一次回调分发，得到 `freq`（Hz）和 `cpu`（耗时占周期比例）
- `freq_lpf`、`cpu_lpf` 用实际周期作 dt、常量 `CUTOFF_HZ` 作截止频率做一阶 RC 低通，滤波与 PID、EKF 共用 `math/filter.hpp`

## Timer

- 两板同写法：BSP 覆盖弱定义 `HAL_IncTick`，照常累加 `uwTick`，再把 DWT CYCCNT 的增量累加到 64 位计数
- 读取时短暂关中断，返回累计值加上未累加的 CYCCNT 差值；中断内可调用
- 约束：关中断或在 SysTick 之上优先级阻塞不得超过一个 CYCCNT 周期（F407 168 MHz 约 25.5 s，H723 520 MHz 约 8.26 s）；调试暂停时两者同停
- Cortex-M7 使能 DWT 前写 `DWT->LAR = 0xC5ACCE55`

## 缓冲

- 固定长度协议数据和收发缓冲优先使用原生数组，命名 `rx_buf`、`tx_buf`
- UART 收发各 512 字节；CDC 每口收发各 2048 字节

## DMA 与 CubeMX

- 硬件 UART 收发统一使用 DMA + 空闲中断，屏蔽半完成中断；不回退到逐字节中断收发
- 时钟、GPIO、DMA 句柄关联及 NVIC/IRQHandler 由 CubeMX 生成，BSP 不手写替代初始化；USB 中断例外，见下文
- `.ioc` 与生成文件由用户操作，AI 给出配置和 USER CODE 片段

### H7（mc02）DMA 与 D-Cache

- DMA1/DMA2 不能访问 DTCM，生成的链接脚本把 .data/.bss/堆栈放在 DTCMRAM
- `bsp/dma_buffer.ld` 先 `INCLUDE` 生成脚本，再以 `INSERT AFTER .bss` 追加 NOLOAD 段 `.dma_buffer` 到 RAM_D1；`toolchain.cmake` 把链接脚本换成它并加 `-L` 搜索目录。INSERT 脚本不能作为第二个 `-T` 传入，否则在主脚本前解析，报 `.bss not found for insert`
- 只把 DMA 收发缓冲放入 `.dma_buffer`，Context 仍在 DTCM；NOLOAD 不清零
- D-Cache 开启、不用 MPU：发送拷贝后启动 DMA 前 `SCB_CleanDCache_by_Addr`，接收通知订阅者前 `SCB_InvalidateDCache_by_Addr`；缓冲 `alignas(32)`、容量为 32 的倍数，避免失效波及相邻数据

### FDCAN（mc02）

- 经典帧 1 Mbps；`HAL_FDCAN_ConfigGlobalFilter` 把标准帧和扩展帧都送入 RX FIFO0，远程帧拒收
- 通知 `RX_FIFO0_NEW_MESSAGE | BUS_OFF`，IT0 中断由 CubeMX 开启；发送 `AddMessageToTxFifoQ`，队列满返回 false
- 接收区按 64 字节准备：HAL 按 FD 长度表拷贝，经典帧 DLC 9–15 会拷贝超过 8 字节
- bus-off 后硬件置位 `CCCR.INIT`；`HAL_FDCAN_ErrorStatusCallback` 计数并清除，经 129 × 11 个隐性位后自动重新入网

### USB CDC（CherryUSB）

- 协议栈为 `third_party/CherryUSB/`（v1.6.1 子集，原样），各板 BSP CMake 只编译 `usbd_core`、`usbd_cdc_acm`、`usb_dc_dwc2` 和 `usb_glue_st`，配置为各板 `bsp/usb_config.h`
- `usb_config.h` 定义 `CONFIG_USB_MEMCPY_DISABLE`：否则 `usb_memcpy.h` 把 `memcpy` 宏替换为 `usb_memcpy`，破坏 C++ 的 `std::memcpy`
- CubeMX：启用 USB_OTG_FS（C 板）或 USB_OTG_HS 内置 FS PHY（mc02）为 Device_Only；不启用 USB_DEVICE 中间件、不勾选 OTG 中断、不生成 `MX_USB_OTG_*_PCD_Init` 调用
- 生成的 `HAL_PCD_MspInit` 提供 GPIO 与时钟，由 CherryUSB `usb_dc_low_level_init` 调用；OTG IRQHandler 由 `usb_glue_st.c` 定义，`cdc_init()` 设置并开启 NVIC
- DWC2 工作在非 DMA 模式，CPU 读写 FIFO，H7 上的 CDC 缓冲也无需 D-Cache 维护
- 描述符常量放在各板 `cdc.cpp` 顶部：VID/PID 默认 CubeMX `0x0483 / 0x5740`，厂商、产品字符串可改；序列号同 CubeMX `Get_SerialNum`，由芯片 UID 生成 12 位十六进制
- C 板 OTG_FS 除 EP0 只有 3 个 IN 端点，只能容纳 1 个 CDC；mc02 为 IAD 复合设备，接口 0 为 `Cdc(1)`、接口 2 为 `Cdc(2)`
- 批量 IN 长度为包长整数倍时补发零长包

### KvStore（FlashDB）

- 源码为 `third_party/FlashDB/`（2.2.0 子集，原样）：`fdb.c`、`fdb_kvdb.c`、`fdb_utils.c` 与 FAL 的 `fal.c`、`fal_flash.c`、`fal_partition.c`
- FlashDB 用 `#include <fdb_cfg.h>`、`<fal_cfg.h>` 取配置，不改源码；各板 `bsp/CMakeLists.txt` 创建 `flashdb` 目标，把本板 `bsp/flashdb/` 加入搜索路径，与 CherryUSB 的 `usb_config.h` 同法
- 各板 `bsp/flashdb/` 放 `fdb_cfg.h`、`fal_cfg.h` 和 Flash 驱动；C 板驱动为上游示例 `fal_flash_stm32f4.c` 原样
- 只有板卡 `kv_store.cpp` 包含 `flashdb.h`；`lib/bsp/kv_store.hpp` 不暴露 FlashDB
- C 板：内部 Flash，分区 768 KiB 偏移、256 KiB，`FDB_WRITE_GRAN 8`
- mc02：板载 W25Q64，OCTOSPI1 Port1（Quad 引脚分配），HCLK 260 MHz 六分频得 43.3 MHz，CS 高电平 3 周期；驱动 `fal_flash_w25q64.c` 用间接模式单线命令（0x03 读、0x02 页编程按 256 字节拆分、0x20 扇区擦除），`HAL_OSPI_AutoPolling` 等待 BUSY 清零，初始化软复位并核对 JEDEC ID 0xEF4017；分区 0 起 1 MiB，`FDB_WRITE_GRAN 1`；CPU 读写 FIFO，无需 D-Cache 维护
- 板卡配置里把 `FDB_PRINT`、`FAL_PRINTF` 定义为空，不输出日志

## 配置注释

- 有 CubeMX 配置要求的外设文件头使用 `/* ... */` 多行注释
- 用表格或短句列出页面、参数、设置值、引脚和时钟推导
- 无专用外设配置的实现不补空泛配置表
