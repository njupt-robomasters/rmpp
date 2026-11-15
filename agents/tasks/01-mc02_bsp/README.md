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

## EL05 MIT 与 CAN2 丢反馈排查（2026-11-14）

- 正确工程是 RM/rmpp/app/jelly_sentry；此前会话错误修改了另一个 RM/jelly_sentry，相关旧工程构建结论不适用于本工程，未回滚旧目录以免覆盖用户工作
- 已确认实际接线共用 CAN2：pitch1 tx/rx=0x10/0x20、pitch2=0x11/0x21、EL05=0x12/0x22，没有 ID 重复；CAN3 为 P630，发送暂注释
- 原 rmpp EL05 仍是扩展帧私有协议，与用户的 protocol_1=2 不匹配，已改为 MIT 标准帧并按手册印刷页51~53核对，详见 docs/motor.md 和 docs/application.md
- 新驱动故障时停止出力，每500ms尝试清错，收到无故障反馈且上层使能后恢复；控制与反馈的16/12位布局均按手册处理
- 参数由用户电机上位机写入并断电重启：CAN_ID=18、CAN_MASTER=34、protocol_1=2、CAN_TIMEOUT=20000
- 已局部编译：jelly_sentry 的 el05.cpp/app.cpp/shooter.cpp 对象，以及 jelly_sentry_debug-shooter、shooter_rub、shooter_shoot 的 app.cpp 对象，均成功且未出现编译警告；未完整链接、烧录或硬件验证。git diff --check通过；本机未找到clang-format，格式为人工检查
- 复现：ninja -C build/jelly_sentry lib/CMakeFiles/rmpp_lib.dir/motor/el05.cpp.obj app/CMakeFiles/rmpp_app.dir/app.cpp.obj app/CMakeFiles/rmpp_app.dir/shooter.cpp.obj；其他三个入口分别 ninja -C build/<preset> app/CMakeFiles/rmpp_app.dir/app.cpp.obj
- EmberProbe config_get 仍返回 BRIDGE_UNAVAILABLE，searchedFrom=RM/uart2can-master、registryEntries=1；尚未获得正确工作区的探针配置、ELF或芯片状态，不能断言两个4310丢反馈的实机原因。需要当前会话工具工作目录同步到rmpp后继续，只读检查contexts里的bus_off_count和monitor、pitch1/pitch2/shoot在线与状态、FDCAN2错误寄存器、FIFO和发送队列
- 实机排查还需确认接入EL05前后电源、波特率、CANH/CANL及终端电阻；未经用户批准不halt、复位、烧录或改寄存器

### CAN2 降载（2026-11-14）

- 按用户指定：pitch1发送250Hz、EL05发送500Hz；pitch2保持1000Hz、小yaw保持总500Hz（A1/94各250Hz），主控控制计算周期不变
- 整车app.cpp分频发送；shooter与shooter_shoot调试入口同步EL05 500Hz。gimbal调试当前只发送pitch2，shooter_rub不发送EL05，未改动
- 正常一问一答时总线预计4500帧/秒，相比原7000帧/秒减少约35.7%；理论占用从78%~95%降至50%~61%。未抓包实测，尚不能认定已修复7025断联
- 脚本can2_rate_update.cjs保存本轮修改；局部编译与最终检查结果见下方续记

- 验证：整车、shooter、shooter_shoot 三个 app.cpp 对象经 Arm GCC 局部编译成功，无编译警告；本次改动范围的 git diff --check 通过。未完整链接、烧录、复位或实机测试。

### 运行时发送周期（2026-11-14）

- 按用户要求在hpp各电机实例旁加入inline volatile uint32_t周期变量：pitch1_send_period_ms=4、pitch2_send_period_ms=1、yaw2_send_period_ms=2、shoot_send_period_ms=2
- 整车、pitch2单轴/通用单电机调试、两个EL05发射调试入口读取对应周期；计数器为uint32_t，控制计算周期保持1ms
- 调试器可写，当前周期计数保留，下一次判断使用新值；0等效1ms，7025建议不小于2ms，周期应小于主控100ms反馈超时
- 尚未烧录或通过探针改写变量，局部编译与检查结果见续记

- 验证续记：整车 app.cpp/gimbal.cpp/shooter.cpp 及 gimbal、gimbal_motor、shooter、shooter_shoot 四个调试入口的对象文件局部编译成功，无编译警告；本次范围 git diff --check 通过；arm-none-eabi-nm 确认四个 send_period_ms 变量均生成符号。未完整链接、烧录、复位或通过探针实际写变量；未找到 clang-format，格式为人工检查。

### 7025 / PCAN 独立总线 ID 干扰分析（2026-11-14）

- 用户报告：只有脉塔电机和 PCAN 时，0x21、0x40、0x41、0x20、0x100 会引起反馈频率浮动；0x50、0x31、0x130、0x26、0x141 不会。补充确认：PCAN 一直交替发送电流控制和位置查询；反馈间隔由稳定 2 ms 跳至十几 ms。干扰帧的帧类型、DLC、内容、发送周期完全相同，仅切换 ID，且一次只测一个干扰 ID。未取得原始 trace，仍需确认请求在线间隔与应答间隔是否同时变化
- 指定资料 `RM/数据手册/L-7025-240626/脉塔7025电机/L-7025-240626.pdf` 仅一页产品规格，适配驱动 MC200、CAN 1 Mbps；同目录另有《伺服电机控制协议V4.3--中文版.pdf》（适用驱动 V3），需结合实际驱动/固件确认适用性
- 协议印刷页 1：标准数据帧、DLC=8，单机控制 0x140+ID、回复 0x240+ID，广播 0x280；不能把 CAN ID 0x20/0x31 等同于 DATA[0] 命令字
- 印刷页 67~69：0x20/index=0x02/Value=1 使能 CANID 滤波器以提高收发效率，保存 FLASH；关闭时可使用 0x280 广播。ID=1 的设置为标准帧 0x141 / DLC8 / `20 02 00 00 01 00 00 00`，同内容从 0x241 应答。示例文字把 DATA[1] 误写为 0x01，实际表格/发送报文均为 0x02；0x01 是清多圈值。手册要求避免刚启动或运动时写入
- 印刷页 66~67：0xB6 可主动循环上报多种指令，间隔单位 10 ms；使能后不再对收到的指令逐条应答。按用户补充优先检查交替问答路径，核对回复命令字，并区分命令应答与 PCAN 界面显示周期
- 分析：若其他参数一致，两组 ID 都低于常见回复 0x241，单纯优先级不能充分解释分组；滤波器/接收处理路径值得优先对照，但不能据此认定固件缺陷或具体掩码。若错误帧随干扰增加，应转查位时序、终端、接线/波形；无错误且滤波器开关显著改变现象，支持接收路径相关
- 两份本地 PDF 均未找到“回复上限 500 Hz”的明确条款；此前 2 ms 建议不能在本次分析中当作已核实厂家硬上限
- 建议验证：固定全部发送参数，仅切换 0x20/0x26 或 0x40/0x50；保存各 10 秒带时间戳 trace，比较命令数、应答数、间隔分布与错误帧；按 DATA[0] 区分同 ID 下不同回复。用户决定是否在静止台架执行一次滤波器设置并复测，不周期写 FLASH
- 后续判据：请求仍稳定但应答减少/停顿，支持电机接收处理、应答发送或链路错误方向；请求本身同时拉长则先查 PCAN 实际发送与总线访问。若合并两种回复每 2 ms 一帧，单种回复正常应约每 4 ms 一帧，不把不同命令混合统计判成单命令漏答
- 本轮只读资料和代码、记录分析；未修改工程代码、连接或操作电机。PDF 渲染核对图放在本任务 out/，未进行硬件验证，根因未确认
