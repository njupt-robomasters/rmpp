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
- 脚本在 `agents/out/01-mc02_bsp/`：`sample.tcl` 采样 PA7，`zero_byte.tcl`、`keepio_off.tcl`、`cpol_cpha.tcl`、`idle_level.tcl`、`idle_long.tcl` 会 halt 并改写 SPI6，运行前须经用户同意

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
- 本轮只读资料和代码、记录分析；未修改工程代码、连接或操作电机。PDF 渲染核对图放在`agents/out/01-mc02_bsp/`，未进行硬件验证，根因未确认

### 电机总线调整（2026-11-15）

- 按用户要求把小 yaw 7025 从 CAN2 移到 CAN3，两个 P630 摩擦轮从 CAN3 移到 CAN2，共用实例使相关调试入口同步生效
- 保留 7025 标准帧 0x141/0x241、P630 VESC 扩展帧节点 0x21/0x22，以及原参数、发送周期与发送注释开关；EL05 和两路 pitch 仍在 CAN2
- 整车与 gimbal、gimbal_motor、shooter、shooter_rub、shooter_shoot 五个调试入口的 rmpp_app 局部编译通过，零警告；imu_calib 检查无须重编译
- 格式检查覆盖修改代码与注释；日志见 out/*_can_swap.log。未完整链接、烧录或硬件操作

### 大小 yaw 同步震荡只读诊断（2026-11-16）

- 使用 EmberProbe 现有采样连接，ELF 为 `build/jelly_sentry/rmpp.elf`，SHA-256 `03734b37577385611fad9d6c099f11c5317132f67eece5b07f072ed4fff828a1`，Bridge 报告与最后烧录记录匹配；未执行 Flash 比对、halt、复位或变量写入
- 采集 200 点、50 ms 间隔，实际跨度 9.989 s；数据见 `out/yaw_oscillation_20261116.csv`。采样期间两台 yaw 持续在线且使能，IMU_MODE；航向目标固定 0.109864 rad，速度输入 0
- 大 yaw 实测范围 2.123~2.564 rad，小 yaw -0.189~0.205 rad；小 yaw 目标在 ±0.698132 rad 间约每 0.45 s 换向，大 yaw 力矩指令约 ±1.95 Nm。IMU yaw 经圆周差分后与编码器合成角变化率相关系数 -0.9993；各电机位置差分与自身速度相关系数分别 +0.9960、+0.9938
- 判断：IMU 航向与两电机合成角的正方向相反，IMU 位置外环形成正反馈，误差绕过 ±π 时改变最短旋转方向，带动两电机同步往返；需要先统一方向，再评价 PID 参数
- 诊断时实时方向为 yaw1=false、yaw2=true，offset 分别 0.332255、5.319590 rad；用户随后确认修改，已将源码方向设为 yaw1=true、yaw2=false，与 IMU 航向统一，保留原始零点与大 yaw 的 KvStore 加载逻辑；待更新固件后人工低输出测试
- 最后一次读取已失能；调试访问无目标状态修改。实时多变量读取存在跨控制周期时间差，相关性按整段数据统计
- 方向修改后整车与 gimbal、gimbal_motor 调试入口的 rmpp_app 局部编译通过，无编译警告；git diff --check 通过。未找到 clang-format，人工核对修改行格式；未完整链接、烧录或执行硬件操作

### EL05 按 old 分支恢复私有扩展帧（2026-11-18）

- 用户指定参考 https://github.com/Joushua233/rmpp/tree/old；已 fetch 核对 old@e7bbcb74a9acb41d673ff5b8c167fbd4f1bc3ab0 的 `rmpp/lib/motor/EL05.*` 和 `rmpp/app/sentry/gimbal.hpp`。该分支实例名为 pitch，使用私有协议、1 Mbps 扩展帧，默认电机/主机节点为0x7F/0xFD
- 当前 EL05 收发改为同一私有协议，保留本工程 CAN2、电机节点0x12、主机节点0x22、5 Nm限幅和原方向；未移植旧云台 PID、零点、行程或默认节点。使能ID为0x03002212、停止为0x04002212、无故障Motor模式反馈为0x02801222
- 运控类型1把16位力矩放入扩展ID，数据区为16位零位置、16位零速度和零Kp/Kd；反馈类型2按16位位置/速度/力矩及0.1℃温度解析，模式和六位故障从扩展ID取得。删除MIT专用warning字段
- 保留当前100 ms使能刷新、断联/未运行时零力矩、500 ms故障清理；私有清错采用类型4停止帧Byte0=1。旧分支的额外力矩反向开关为false，当前沿用config.is_invert即可
- 配套电机参数为CAN_ID=18、CAN_MASTER=34、protocol_1=0；之前写入记录是protocol_1=2，需用户在电机上位机核对并调整后断电重启。本轮未通过探针修改电机参数或操作硬件，实际协议和运转结果待台架核对
- ARM局部编译通过且无编译警告：整车el05.cpp、app.cpp、shooter.cpp，以及shooter、shooter_rub、shooter_shoot三个调试入口rmpp_app。未完整链接或烧录
- 新增 `el05_protocol_test.cpp`，编译真实EL05与Motor源码，只模拟CAN与时钟。通过标准帧/错误节点/错误类型/远程帧/长度筛选、16位反馈与温度解析、正负力矩及5 Nm限幅、非有限输入、未连接/未运行/超时保护、周期使能、六位故障与清错、恢复和方向反转检查。主机Zig/Clang编译有4条Eigen与C++23库的既有弃用警告；ARM编译零警告
- 本次手写代码经clang-format 22.1.0整理，格式检查和diff空白检查通过；测试可执行文件位于本任务out/，未入库

主机测试复现（支持C++23的主机编译器）：

```sh
c++ -std=c++23 -O0 -DEIGEN_DONT_VECTORIZE -Wno-unknown-pragmas -Ilib -Ilib/third_party/eigen-3.4.0 agents/tasks/01-mc02_bsp/el05_protocol_test.cpp lib/motor/el05.cpp lib/motor/motor.cpp -o agents/out/01-mc02_bsp/el05_protocol_test.exe
agents/out/01-mc02_bsp/el05_protocol_test.exe
```

### EL05 上位机能打开适配器但搜不到电机（2026-11-19）

- 用户确认失败环节为设备搜索。工作区整车app.cpp的CAN发送已由用户注释，保留该修改；实际已烧录固件与电机侧协议待核对
- 本地手册印刷页37第4.2.4节说明：切换为MIT后，需通过CAN盒发送MIT指令8切回协议。结合此前protocol_1=2记录，优先验证该方向
- 印刷页53指令8：按之前电机CAN_ID=18，使用1 Mbps、经典标准数据帧、ID 0x12、DLC8、`FF FF FF FF FF FF 00 FD`，其中第7字节0表示私有协议，重新上电生效。建议断电后将总线简化为单EL05与CAN适配器，再发送一次并给电机断电重启后重新搜索
- 指令待用户通过CAN工具执行；本轮只核对手册和代码，未发CAN命令、改参、烧录、复位或操作硬件。若恢复后仍搜不到，需原始收发帧与错误计数继续核对波特率、终端和连接

### EL05 私有协议下单片机离线（2026-11-19）

- 用户已确认上位机可连接、protocol_1=0，CAN_ID=18、CAN_MASTER=34，与MCU配置相同
- 已观察：EmberProbe通过现有采样连接只读检查，所选ELF为build/jelly_sentry/rmpp.elf，SHA256 `bf4d34766c2349d043723f7b23025cf964c1cf70b72053915e184410c1883330`，构建时间11月18日23:40:40；Bridge报告与最后烧录记录匹配（11月18日23:41:05），未执行Flash逐字节比对
- 现场值：shoot.is_connect=0、callback_timer.last_cycles=0，mode/温度/原始反馈均0；CAN2已注册包括EL05在内的5个接收回调，HAL状态为2（BUSY）、ErrorCode=0、bus_off_count=0，位时序Prescaler=5、Seg1=19、Seg2=5；VT13在线C挡
- 已定位的代码原因：整车send_can_cmd()内shoot.send()被注释；当前ELF保留EL05接收回调，EL05::send()及发送周期/分频符号被链接裁剪。默认未主动上报时，缺少周期命令将使EL05反馈在线检测无法建立；恢复发送后的实际应答还需新固件验证
- 已仅恢复EL05按shoot_send_period_ms（默认2 ms）发送，保留pitch1/pitch2、yaw2和rub1/rub2的现有发送注释。C挡下发送停止帧仍应获得状态反馈
- 验证：整车app.cpp ARM对象编译成功、格式检查与diff空白检查通过；编译存在用户注释yaw2发送后留下的1条yaw2_divider未使用警告。本轮未完整链接、烧录、复位、halt或修改硬件变量，现有采样保持原状
- 下一步：用户构建并更新固件后，读取shoot.is_connect及反馈计时器，核对实际收到的私有扩展帧；无故障复位状态反馈预期0x02001222，Motor模式预期0x02801222

### EL05 与双 rub 方向调整（2026-11-19）

- 用户实测EL05及rub1/rub2均运行反向，要求在hpp显式暴露is_invert配置；已在shooter.hpp的三个实例中均配置is_invert=true，rub1/rub2从默认false改为显式true，shoot由false改为true
- 复用现有方向修正：EL05发送力矩与Motor反馈同步反号，Vesc速度指令与Motor反馈同步反号；观察入口为三个实例各自的config.is_invert。保留协议、节点、PID及现有发送开关
- 验证：整车与shooter、shooter_rub、shooter_shoot三个调试入口的rmpp_app局部编译成功，7个对象无编译警告；头文件格式及diff空白检查通过。未完整链接、烧录或通过探针改写运行参数，实际方向待用户更新固件核对

### EL05 再次停转只读诊断（2026-11-19）

- 当前所选ELF构建时间20:39:44，SHA256 `98522f09a1e867b4ac430ecce3ae0b63bb193f546b6caddc4a703b2d7c5b3618`，Bridge报告匹配20:39:57最后烧录记录；经临时探针连接读取，未修改共享采样开关
- 已观察：初次VT13为S挡、trigger=1、pitch=0，机构已使能但目标频率/力矩均0，EL05离线、mode=RESET、fault=0；原始位置和22℃温度保留历史反馈。随后12点采样中EL05反馈计时固定为90518763301，使能计时持续更新，CAN2接收计时持续更新，说明EL05有效反馈中断而主循环仍在运行
- 后段30点、100 ms间隔采样中EL05全部在线，反馈间隔约1.85~2.15 ms，mode=RESET、fault=0；VT13处于N后转C，trigger/pitch/目标频率/力矩均0，符合停转输入。两路pitch该段离线，EL05与其他设备反馈的变化需结合现场接线、供电与上位机是否连接继续核对
- CAN2的HAL错误码与bus_off_count采样均0；AutoRetransmission实测为0（关闭）。尚未取得总线原始帧或错误计数寄存器，不把反馈中断归因为具体CAN故障
- 运行时shoot.config.is_invert=0，rub1/rub2均1；当前源码shoot的显式is_invert已由用户移除，保留该修改。方向配置与先前提交请求后的源码有变化，本轮未再次改方向
- 标量采样归档out/el05_no_turn_20261119.csv；已请求用户确认上位机是否断开，并在可运行时保持S挡、扳机与左摇杆以继续定位。该操作状态尚未获得回复
- 最后一次采样OpenOCD启动失败；查询sampling_status为stopped后重试一次variables_read仍返回OPENOCD_START_FAILED，USB枚举仍能看到Horco CMSIS-DAP v2序列号d8cd1ad9。停止重复连接，待恢复探针/目标连接再核对持杆状态。本轮未改工程代码、烧录、复位、halt或改写内存/寄存器

### USB-CAN 断开后的持杆与C挡对照（2026-11-19）

- 用户确认USB-CAN已断开，并保持S挡、扳机及左摇杆；Ember重新读取成功，所选ELF与上节同一SHA，Bridge仍报告匹配。期间未执行复位；新读数中的系统累计周期比上节小，只按当前每段数据分析
- S挡30点/100 ms采样：trigger持续1，pitch达到+1及-1，bullet_freq.ref达到+20及-20，机构/电机使能为1，heat_protect=0；EL05全程is_connect=0、state=STOP、torque_ref=0。有效反馈计时固定10774699819，使能刷新计时持续更新。已定位直接停转条件为有效反馈超时触发机构STOP与驱动零力矩保护
- 同轮单次读数两路pitch在线、反馈时间持续推进；CAN2状态BUSY，HAL ErrorCode=0，bus_off_count=0，AutoRetransmission=0。当前源码同时启用pitch1 250 Hz、pitch2 1000 Hz、rub1/rub2各1000 Hz、EL05 500 Hz请求
- 用户随后切C挡，30点采样仍为EL05离线、两路pitch在线；EL05最后反馈时间已更新到64739511907、mode=Motor(2)、fault=0，随后整段不再更新。EL05应答呈间歇状态，C挡本轮未稳定恢复
- 优先待验证方向：关闭自动重发时仲裁丢失可取消EL05请求或使能帧。Bosch M_CAN User Manual v3.3.1第3.1.7节说明DAR下仲裁失败/发送错误后取消发送：https://www.bosch-semiconductors.com/media/ip_modules/pdf_2/m_can/mcan_users_manual_v331.pdf 。现有HAL错误码和bus-off计数不覆盖该类仲裁取消；尚未读取TXBTO/TXBCF或取得总线trace，根因保留待验证
- 进一步寄存器只读诊断需在Ember绑定匹配SVD，当前svd为空，未调用寄存器读写、修改配置或绕过工具。对照可由用户在CubeMX启用FDCAN2 Auto Retransmission并生成、更新固件后进行；.ioc及生成代码保持由用户操作
- 采样归档out/el05_held_and_C_20261119.csv。本轮仅只读诊断与记录，未改工程代码、烧录、复位、halt或改写运行参数；现有共享采样状态未主动改变
