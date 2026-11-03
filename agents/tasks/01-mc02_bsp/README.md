# 01 mc02 BSP 适配与 C 板优化

## 目标

- mc02（STM32H723）适配 FDCAN、串口、BMI088 与加热、WS2812、蜂鸣器、双 CDC、可控电源
- C 板去掉公共 Pwm/Spi，改为 Rgb、Buzzer、Imu 板载外设类；USB 改用 CherryUSB
- lib：共用一阶 RC 低通、`lib/imu` 改为 `lib/attitude`、蜂鸣器播放器改为 `Melody`

长期结论已写入 `agents/docs/bsp.md`、`architecture.md`、`imu.md`、`build.md`、`music.md`

## 复现构建

CLion 自带工具未加入 PATH 时：

```sh
C='/d/JetBrains/CLion 2026.2.2/bin'
export PATH="$C/cmake/win/x64/bin:$C/ninja/win/x64:$PATH"
for p in examples-cdc infantry examples-imu examples-gimbal examples-rgb examples-melody; do
    cmake --preset $p && cmake --build build/$p
done
```

格式化使用 `plugins/clion-radler/DotFiles/windows-x64/clang-format.exe`

## 结论（2026-10-31）

- 上述预设及 chassis、motor、rc、referee、shooter 示例均编译链接通过，本任务代码 0 警告；motor 示例有 2 条既有的未使用函数警告
- sentry 因 `motor/lk/mf9025.hpp` 已不在 HEAD 而编译失败，与本任务无关
- mc02 示例：DTCM 14.1 KB、RAM_D1 4 KB（UART DMA 缓冲）、Flash 184 KB
- 未烧录、未上板验证

## WS2812 不亮（2026-11-02）

- 现象：正常运行不亮，打断点闪亮一下后不再更新
- 原因：Keep IO State 开启时 MOSI 始终推挽。发完、SPE 仍开时 MOSI 保持最后发出的位；HAL `SPI_CloseTransfer` 关闭 SPE 后，MOSI 变为 FIFO 中某个已发字节的首位。旧编码 0xC0/0xF8 首位为 1，线路常高，WS2812 无法锁存
- 实测（`idle_level.tcl`、`idle_long.tcl`）：MSB first 时关 SPE 后取字节最高位，LSB first 时取最低位，即该字节首发位；1、2 字节帧取第 0 字节，24 字节帧取第 16 字节（正好倒数第 8 个，推测与 SPI6 的 8 字节 FIFO 有关，未核对手册）
- 处理：保留 Keep IO State 推挽输出，编码右移一位为 0x60/0x7C，与达妙例程一致；待用户烧录确认
- 脚本在 `out/`：`sample.tcl` 采样 PA7，`zero_byte.tcl`、`keepio_off.tcl`、`cpol_cpha.tcl`、`idle_level.tcl`、`idle_long.tcl` 会 halt 并改写 SPI6，运行前须经用户同意

## 实现中的坑

- CherryUSB `usb_memcpy.h` 把 `memcpy` 定义为宏，C++ 的 `std::memcpy` 报错；`usb_config.h` 定义 `CONFIG_USB_MEMCPY_DISABLE`
- GNU ld 的 `INSERT AFTER .bss` 脚本无论作第二个 `-T` 还是隐式脚本，都在主脚本之前解析，报 `.bss not found for insert`；改为包装脚本先 `INCLUDE` 生成脚本再 INSERT
- H7 HAL `HAL_FDCAN_GetRxMessage` 按 FD 长度表拷贝，经典帧 DLC>8 会写越 8 字节缓冲，接收区按 64 字节准备

## 待用户操作

- mc02 `Core/Src/main.c` USER CODE 2 加入入口调用（与 C 板相同）
- 确认两板 CubeMX 的 OTG 中断保持不勾选、不生成 `MX_USB_OTG_*_PCD_Init` 调用
- 上板验证建议顺序：Timer/Rgb/Buzzer → Imu 加热 → UART 回环 → CAN（先 `set_power(OUT_5V, true)`）→ CDC 枚举与回环
