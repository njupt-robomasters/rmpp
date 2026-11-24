# MiniPC 通信与导航

## 通道与接口

- mc02 `app/jelly_sentry` 使用 `Aim({.cdc = Cdc(1)})` 自瞄和 `Nav({.cdc = Cdc(2)})` 导航
- C 板 `app/steer_sentry` 只有一路 CDC，`Nav({.cdc = Cdc(1)})` 与 Aim 配置共用端口；底盘自主控制与 jelly_sentry 相同，坐标转换和 flag 仲裁见下节，flag 0 因未配跟随 PID 只平移不旋转
- Nav 构造注册 USB 接收回调；对象在回调使用期间持续存活，现有应用均为全局实例
- `nav.update()` 在主循环的 `chassis_loop()` 前调用，公开只读输出为 `is_connect`、`vx`、`vy`、`yaw`、`action_flag`
- 已移除旧 `chassis_speed`、`target_position` 接口及两处应用只在本地赋值的 GO_CENTER/GO_HOME 占位流程；目标点与返航由上位机规划，目标点/状态上行协议待约定

## 自瞄收发

- 下行 18 字节，依次为 `A5 5B`、识别标志、yaw、pitch、开火标志、距离、CRC16。两个标志各占 1 字节，非 0 为真
- 上行 21 字节，依次为 `A5 5A`、yaw、pitch、roll、红方标志、裁判实测初速、CRC16。红方标志占 1 字节，红方为 1，蓝方为 0
- 浮点均为小端 IEEE 754 float32，角度用度，距离用 m，初速用 m/s。线上 pitch 抬头为正，Aim 内部以右手系弧度表示。两向 CRC16 均用 MODBUS，覆盖帧头和 CRC 之间的字节，低字节先传
- 帧结构用 `packed` 固定字段偏移，浮点和 CRC 成员按值访问。主机协议测试用固定字节帧检查布局和浮点表示
- 接收按“协议接收”统一写法（`../docs/conventions.md`）解析，CRC 通过后保存 `raw_rx`，直接更新 `is_connect`、`is_detect`、`yaw`、`pitch`、`is_fire`、`distance`；字段按载荷原样映射，识别标志为 0 时开火标志仍按载荷发布，由上位机保证无目标时不开火。两处哨兵 app 用短 `IrqLock` 读取同一帧供当轮云台和发射使用
- `aim.update()` 只做断联检测，默认 100 ms 无有效帧后目标清零，`raw_rx` 保留
- 发送帧 `raw_tx` 为私有成员：`set_imu()`、`set_referee()` 换算后直接写入，`send()` 补 CRC 后交给 CDC 复制发出，SWD 可查看最近一次发送内容。应用每 5 个主循环周期设置后调用 `aim.send()`，1 kHz 主循环下名义 200 Hz

## 导航下行协议

固定 17 字节，小端 IEEE 754 float32：

| 偏移 | 字节数 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 2 | SOF | A5 5A |
| 2 | 4 | vx | 雷达所在大 yaw 系前向速度，m/s |
| 6 | 4 | vy | 大 yaw 系左向速度，m/s |
| 10 | 4 | yaw | 上位机角度，rad；当前解析和保存，具体控制用途待约定 |
| 14 | 1 | action_flag | 0 随头，1 允许小陀螺；当前 pitch1 全程失能 |
| 15 | 2 | CRC16 | MODBUS，低字节在前 |

Aim 与 Nav 共用 `lib/minipc/crc16_modbus.hpp`。CRC 初值 0xFFFF，反射多项式 0xA001，无结果异或；Nav 覆盖偏移 2–14 共 13 字节。独立于裁判系统的 CRC16。参考帧（vx=1、vy=0、yaw=0、flag=0）：

```text
A5 5A 00 00 80 3F 00 00 00 00 00 00 00 00 00 9A E6
```

Nav 按“协议接收”统一写法解析：`rx_fifo` 容量为一帧，帧头不符或 CRC 不符时丢弃 1 字节重新对齐，CRC 通过后整帧出队、保存私有 `raw` 并直接更新公开指令。浮点有限性和 flag 范围不检查，按载荷原样发布。当前导航协议只有下行。

断联只在 `update()` 检查：默认 100 ms 无有效帧则离线，公开指令清零，`raw` 保留供 SWD 查看；坏帧持续到达也会超时。超时不清 `rx_fifo`，避免主循环与接收中断同时改写；旧半帧与新字节只有凑成 CRC 正确的整帧才会被接收。应用依据 `is_connect` 判断有效性，清零后的 flag=0 不会授权自主随头。底盘读取 `vx`、`vy` 时未加锁，两次读取之间若到达新帧，会混用相邻两帧的分量。

## 哨兵控制链路与坐标

导航只在自主控制（`sentry_active()`：哨兵身份比赛中，或 VT13 fn 锁存测试）的 `control_by_auto()` 中生效，VT13 操控量此时不叠加；非自主时底盘只由 VT13 控制。自主模式下 Nav 或任一 yaw 反馈离线，平移和旋转指令为零，并设置 DETACH_MODE。

三个平面坐标系均为 X 前、Y 左，逆时针为正：

- B：底盘本体
- L：固连大 yaw 的雷达系，按雷达安装轴与大 yaw 对齐处理
- H：小 yaw 后的 IMU 头/枪口云台系，人工速度及 ChassisMatrix::set_speed 使用此系

令 q1 为大 yaw 相对底盘角，q2 为小 yaw 相对大 yaw 角，总角 q=q1+q2：

```text
上位机 L 系速度 → H 系：v_H = R(-q2) v_L
底盘内部 H 系 → B 系：v_B = R(q) v_H
合成结果：v_B = R(q1) v_L
```

`q2` 取 `gimbal.yaw2.measure`，总角取 `gimbal.yaw.ecd.measure`。小 yaw 左转 90° 时，雷达系向前对应云台系向右。雷达与大 yaw 轴的实际安装偏角需在台架核对。

| 有效 flag | 模式 | 旋转策略 |
| --- | --- | --- |
| 0 | FOLLOW_MODE | 导航 wr 输入为 0，由跟随 PID 根据总云台相对底盘角对准 IMU 头 |
| 1 | DETACH_MODE | 关闭随头，平移速度 0→5 m/s 时，小陀螺输入 120→60 rpm；插值速度限在此范围 |

模式随最新有效 flag 持续设置。平移每轴限幅 ±5 m/s，旋转输入限幅 ±120 rpm，底盘继续执行原有轮电机闭环和超电功率控制。yaw 字段目前仅供协议和 SWD 观察。

## pitch1 固定失能

- `GimbalFold::set_enable()` 对 pitch1 固定调用 `set_enable(false)` 并写零原生输出
- 每次 `GimbalFold::update()` 再固定 pitch1 失能，更新反馈、清空 pitch1 两级 PID，目标跟踪实测，最终 `set_output(0)`；pitch1 位置环和重力补偿输出计算已撤除
- 主应用保留默认 4 ms 的 `pitch1.send()`；DmMotor 在失能分支发送 `FF FF FF FF FF FF FF FD`，CAN ID 0x10，输出目标为零
- 导航 flag 和总云台使能均不会启用 pitch1；其他三轴保持现有控制
- 软件检查与主机测试覆盖失能状态、零电流/力矩目标和失能帧。实机是否已运行新固件及物理输出状态由用户更新固件后核对

验证脚本与本轮结果见 `../tasks/15-minipc_nav/README.md`。
