# 轮腿裁判系统 CAN 转发

## 目标

leg_down 的 USART1 收发裁判系统（电源管理模块）串口，经 CAN1 与上板 Referee 互转。CAN ID 与 leg_protocol 一样按发送板命名：0x00 为下板发送（串口 → CAN），0x01 为上板发送（CAN → 串口）；RefereeRelay 类内按本板视角写作 `tx_id`、`rx_id`，缓冲区为 `to_can`、`to_uart`。Referee 只走 CAN、只收常规链路；图传链路 0x0311 交给 VT13 解析并提供注册回调。长期结论见 `docs/application.md`“轮腿上下板通信”“遥控器映射”

## 改动

- `lib/container/fifo.hpp`：新增环形队列 `Fifo<T, N>`，RefereeRelay、Referee、RefereeSender、VT13 共用；解析器先用 `[]` 检查帧头，收齐后 `peek` 到局部连续数组做 CRC 和解包
- `lib/referee/referee_relay.*`：新增 `RefereeRelay`，`to_can` 1024 B、`to_uart` 256 B，接收中断写入，`send()` 在主循环取出（`IrqLock` 保护）；`to_can` 每次至多发 1 帧 8 字节，`to_uart` 在串口空闲时一次发出
- `lib/referee/referee.*`：`Config::bus`（`std::variant<Can, Uart>`）改为 `Config::can`，删除 UART 分支、0x0310/0x0311 解析与 `add_0311_callback`；载荷上限 300 改为常规链路最长的 118 B（0x0301），接收环形 256 B，`tx_buf` 与局部帧数组 127 B；`packets.hpp` 删除仅供 0x0310 使用的 `ClientData`
- `lib/rc/vt13.*`：同一缓冲区按帧头分别处理 0xA9 0x53 遥控帧与 0xA5 裁判帧，接收环形 128 B，局部帧数组 69 B（0x0311 最长帧）；新增 `add_0311_callback()`
- leg_down：`app.hpp` 定义 `referee_relay`；`app.cpp` 奇拍第 3 帧在 0x003 与下板裁判帧 0x00 之间轮流，0x003 由 500 Hz 降到 250 Hz
- sentry、referee 示例：`{.bus = Can(1)}` 改为 `{.can = Can(1)}`

## 收发拆分（2026-11-20）

- `Referee` 只收：删除 `send()`、`tx_id` 与发送缓冲；`deserialize` → `parse`，`frame_timer` → `connect_timer`，删除无读者的 `hurt_sequence`；0x0301 只解析队友的 0x0200~0x02FF；删除常规链路收不到的 0x0302/0x0306/0x0309（2027 已无）、0x0305/0x0307/0x0308（机器人发出）、0x0A01~0x0A06（雷达无线链路）及对应结构体
- `RefereeSender`（`referee_sender.*`）：原 UiControl 图形构造与绘制接口并入，另加 `send_sentry_command()`（0x0120 → 0x8080）、`send_custom_info()`（0x0308，UTF-16LE 15 字）；20 个请求的队列，组帧时补发送者与本机选手端接收者（robot_id + 0x0100），本机 ID 未知时暂停，无选手端（工程、哨兵等）时丢弃发往本机选手端的 UI；0x0301 间隔 ≥ 1/30 s；同子内容、首图形同名的 MODIFY 原位替换，替代原写死的 0x160/0x170 名称判断；`clear_ui()` 只删 UI 请求
- 图形编码改为位段结构体 `Figure` 直接拷贝；修正椭圆、圆弧半轴写入 details_c/d 的问题（协议为 details_d/e）；浮点数放大 1000 倍改为四舍五入（原截断使 1.234 显示为 1.233）
- `RobotUi`（`robot_ui.*`）：按“谁持有谁更新”，`RefereeSender` 由 app 持有并 `send()`，`init(sender)`、`update(sender, display)` 调用时传入；app 启动时 `init()` 排入初始布局，删除 `initialized` 与 30 Hz `dashboard_timer`（由 MODIFY 合并限制积压）；公开成员改为 `update(const Display&)`，Config 字段 `max_*` → `*_max`；删除重复 `set_arc` 与 `is_bullet_speed`，`bullet` 数值直接转为整数显示
- sentry：`misc.hpp` 定义 `referee_sender` 与 `ui({})`，`app_main()` 启动时 `ui.init(referee_sender)`；`update_ui()` 一次填写 `Display`，弹频不再由 `shooter_loop()` 写 UI

## 复现

```sh
python -I agents/13-referee_relay/test_fifo.py
python -I agents/13-referee_relay/test_referee_relay.py
python -I agents/13-referee_relay/test_referee_sender.py
python -I agents/12-vt13_control/test_vt13.py
python -I agents/13-referee_relay/compile_arm.py
```

- `test_fifo.py`：单元素和批量 `push` 均在队满时弹出最旧元素。覆盖空批次、超容量批次、被挤出元素计数、回绕、peek/pop 和下标；Referee 连续 40 帧使接收环形多次回绕、超过 118 B 载荷丢弃
- `test_referee_sender.py`：图形位段与协议表逐位比对、组帧与 CRC、0x0301 30 Hz 间隔、0x0308 编码与不限频、0x0120 位段、本机 ID 未知暂停、无选手端丢弃 UI、MODIFY 合并、`clear_ui()` 保留 0x0120；CAN 忙时已组帧保留、其余请求只保留最新 20 个；RobotUi 初始布局和仪表的顺序与数值
- `test_referee_relay.py`：编译实际 RefereeRelay、Referee、RefereeSender、CRC，下板接 Can(1)、上板接 Can(2)，由测试搬运 CAN 帧；覆盖下板 0x00 帧 5 字节分包后 0x0201/0x0202 解析、CAN 忙时保留、上板 RefereeSender 0x01 帧 0x0301 拼接还原、串口忙时缓存、ID 过滤；溢出后逐字节核对 `to_uart` 保留最新 256 B、`to_can` 保留最新 1024 B
- `test_vt13.py` 新增：遥控帧、60 B 0x0311、0x0310、CRC 损坏的 0x0311、遥控帧混合后按 7 字节分包送入，只回调一次完整 0x0311 且后续遥控帧照常解析；61 B 载荷不回调
- `compile_arm.py`：借用 `build/leg-leg_down`、`build/leg-leg_up`、`build/sentry` 的 compile_commands 参数，ARM GCC 单独编译涉及的 lib 与 app 文件，未执行完整固件构建

## 流量估算

- BSP UART 收发缓冲保持 512 B：接收空闲中断或满时回调，各路帧远小于 512；发送须容纳以后 VT13 的 0x0310（309 B）

- 2027 协议常规链路定频报文（步兵）约 0.94 kB/s；下板 0x00 容量 250 帧/s × 8 B = 2 kB/s，触发类报文突发由 1024 B 缓冲区吸收
- 上板 0x01 主要为 UI 0x0301，至多 30 Hz × 16 帧；串口 115200 约 11.5 kB/s，容量足够
- CAN1 现有负载按电机反馈频率估算约 85–95%（1 Mbps），转发再增加约 2%（下板 0x00）至 6%（UI 满速时上板 0x01）；0x00/0x01 优先级最高

## 待定与上车核对

- leg_up 尚未接入 `Referee({.can = Can(1)})`；接入后需把 `robot_status.chassis_power_limit`、`power_heat_data.buffer_energy` 交给超电，并安排 `referee.update()`；需要 UI 时再加 `RefereeSender` 及 `send()`（上板 CAN1 每拍只发 1 帧，有空邮箱）
- 上车核对 UI：椭圆/圆弧半轴字段已按协议修正，原布局的弧线显示可能变化，需在选手端确认位置与粗细；sentry（ID 7/107）无选手端，UI 请求会被丢弃
- 0x0310 发送接口未实现，需要时加在 VT13
- 裁判串口实际接在 leg_down C 板 USART1（PA9/PB7）；CubeMX 已配置 115200 8N1 与收发 DMA
- 上车用 SWD 观察 `referee_relay.to_can_drop`、`to_uart_drop` 应为 0，上板 `referee.is_connect` 为真；必要时用 CAN 分析仪核对总线负载
## 裁判协议版本

`lib/referee/packet.hpp` 对应 2026 V1.3.1（20260519），是当前收发器使用的版本；`lib/referee/packet2027.hpp` 对应 2027 V1.0.0，覆盖相同的模块职责。切换赛季时同时更新 `referee.hpp`、`referee_sender.hpp` 的包含文件，并核对应用对赛季字段的访问。同一固件统一使用一个版本。`test_packet_layout.py` 使用 ARM GCC 核对两版包长和关键偏移。
