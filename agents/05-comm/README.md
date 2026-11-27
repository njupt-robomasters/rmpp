# 05 通信与协议

状态：Referee 转发与 MiniPC 协议已并入 master，待上板核对

## 任务记录

- `referee_relay/`：leg_down 裁判串口经 CAN 与上板互转；`Referee`、`RefereeSender`、`RobotUi` 拆分与 UI 修正
- `protocol_style/`：Nav、Aim、VT13、Referee 接收统一写法和命名
- `minipc_protocol/`：当前自瞄与导航协议（A1–A4 帧、四元数、目标角/目标点）、leg_up 接入自瞄
- `nav/`：早期 17 字节导航协议与坐标转换，协议已由 `minipc_protocol` 替代
- `minipc_aim/`：本地历史自瞄协议与上下行验证，当前生产协议检查使用 `nav/verify.py` 和 `minipc_protocol/`

## 关键接口

- `Referee` 直接收 CAN；leg_down 用 `RefereeRelay` 在裁判串口与 CAN 0x00/0x01 之间转发；图传链路 0x0311 归 VT13
- 自瞄：下行 `A1` 17 B（识别/开火标志、yaw、pitch、距离），上行 `A2` 24 B（红方、弹速、四元数）
- 导航：下行 `A3` 24 B（过洞标志、vx、vy、相对转角、x、y），上行 `A4` 15 B（大 yaw 编码器角、目标 x、y）
- 浮点为小端 float32，CRC16 MODBUS 覆盖帧头至 CRC 前；接收统一按 1 字节帧头对齐，帧头或 CRC 不符各丢 1 字节

## 文档

- `minipc.md`：通道、帧结构、断联与坐标链路
- 根 `docs/minipc_protocol.md`：发给上位机的对接文档

## 待办

双 CDC 枚举与隔离、坐标系方向、人工接管与超时、UI 显示、`referee_relay` 丢字节计数，详见 `../TODO.md`
