# MiniPC 通信与导航

对外对接文档（发给上位机同学）见根目录 `docs/minipc_protocol.md`。

## 通道与接口

- mc02 `app/jelly_sentry` 使用 `Aim({.cdc = Cdc(1)})` 自瞄和 `Nav({.cdc = Cdc(2)})` 导航
- C 板 `app/leg/leg_up` 的 `minipc.cpp` 在 CDC1 复用 `Aim`，仅收发自瞄协议
- C 板 `app/steer_sentry` 的 Aim 与 Nav 共用 CDC1，自瞄与导航上行错拍各200 Hz；jelly_sentry 双 CDC 上行错拍各250 Hz
- Nav 构造注册 USB 接收回调；对象在回调使用期间持续存活，现有应用均为全局实例
- 导航断联检查在 `chassis_loop()` 前完成；jelly_sentry 由 `minipc_loop()` 调用 `nav.update()`。公开只读输出为 `is_connect`、`is_through_hole`、`vx`、`vy`、`yaw`、`x`、`y`
- 两台哨兵均由下位机在中心点与家之间选择目标点写入 `A4`，上位机规划路径：`steer_sentry` 血量 ≤120 回家、≥400 去中心；`jelly_sentry` 在 `sentry.cpp` 用 `is_go_home` 维护战术状态，正赛或测试模式下默认发中心 `(6.0, -3.8)` m，不依赖裁判 RFID，收到有效血量后 `<100` 发回家点 `(0.15, -0.15)` m 直至回满 `maximum_HP` 再出击

## 自瞄收发

- 下行 17 字节：`A1`、识别标志、开火标志、yaw、pitch、距离、CRC16。两个标志各占 1 字节，非 0 为真
- 上行 24 字节：`A2`、红方标志、裁判实测初速、四元数 x y z w、CRC16。红方为 1，蓝方为 0
- 浮点均为小端 IEEE 754 float32，角度用度，距离用 m，初速用 m/s。线上 yaw 俯视逆时针为正、pitch 向下为正，与内部 ZYX 右手系同号，收发都按载荷直接映射。四元数为最终 IMU 头姿态、机体到世界，取自 `attitude.q`
- 两向 CRC16 均用 MODBUS，覆盖帧头至 CRC 前的字节，低字节先传
- 帧结构用 `packed` 固定字段偏移，浮点和 CRC 成员按值访问。主机协议测试用固定字节帧检查布局和浮点表示
- 接收按“协议接收”统一写法（`agents/02-conventions/conventions.md`）解析：1 字节帧头不符或 CRC 不符各丢 1 字节重新对齐。CRC 通过后保存 `raw_rx`，直接更新 `is_connect`、`is_detect`、`yaw`、`pitch`、`is_fire`、`distance`；字段按载荷原样映射，识别标志为 0 时仍发布开火标志，上位机应在无目标时清零。两个哨兵应用使用短 `IrqLock` 获取同一帧供当轮控制使用
- `aim.update()` 只做断联检测，默认 100 ms 无有效帧后目标清零，`raw_rx` 保留
- 发送帧 raw_tx 为私有成员，set_attitude/set_referee 设置载荷，send 补 CRC 并交给 CDC。jelly_sentry 名义250 Hz，steer_sentry 和 leg_up 名义200 Hz
- `leg_up` 的 `minipc_loop()` 读取最近一次姿态、裁判机器人 ID 和实测弹速以填充上行，并检查断联；`app.cpp` 每 5 拍调用 `aim.send()`。底盘、云台和发射模块各自直接读取 `aim`；接收中断若发生在多字段读取之间，可能读到相邻帧的字段。VT13 `is_aim` 且 `is_detect` 时云台使用视觉世界角，否则用摇杆与鼠标速度。自瞄模式下有目标且 `is_fire` 为 1 才放行人工弹频；松开人工发射输入即停拨弹

## 导航收发

下行 24 字节、上行 15 字节，两向 yaw 均用 rad，浮点为小端 IEEE 754 float32：

| 偏移 | 字节数 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 1 | SOF | `A3` |
| 1 | 1 | is_through_hole | 过洞为 1，其余为 0 |
| 2 | 4 | vx | 雷达所在大 yaw 系前向速度，m/s |
| 6 | 4 | vy | 大 yaw 系左向速度，m/s |
| 10 | 4 | yaw | 相对大 yaw 当前朝向的期望转角，rad；俯视逆时针为正 |
| 14 | 4 | x | 上位机自定位的当前坐标 x，m；当前仅保存 |
| 18 | 4 | y | 当前坐标 y，m；当前仅保存 |
| 22 | 2 | CRC16 | MODBUS，低字节在前 |

上行 `A4`、大 yaw 编码器角、目标 x、目标 y、CRC16：偏移依次为 0、1、5、9、13。两台哨兵都用 `set_yaw_ecd(gimbal.yaw1.measure)` 写入大 yaw 相对底盘的实测角（rad，俯视逆时针为正）；单独转动小 yaw 时该字段保持不变。`set_target(x, y)` 写入目标点，`send()` 补 CRC 并发送。

下行 `nav.yaw` 原样保存期望转角。`0` 表示保持大 yaw 当前朝向，`+π/2` 表示从当前朝向逆时针转 90°。目前两个哨兵应用均未使用该字段驱动大 yaw。

Aim 与 Nav 共用 `lib/minipc/crc16_modbus.hpp`。CRC 初值 0xFFFF，反射多项式 0xA001，无结果异或；两向均覆盖从帧头到 CRC 前的全部字节。独立于裁判系统的 CRC16。参考帧（过洞=0、vx=1、其余 0）：

```text
A3 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 8A 78
```

Nav 按“协议接收”统一写法解析：`rx_fifo` 容量为一帧，1 字节帧头不符或 CRC 不符时丢弃 1 字节重新对齐，CRC 通过后整帧出队、保存私有 `raw_rx` 并直接更新公开指令。浮点有限性和标志取值不检查，按载荷原样发布。

默认100 ms无有效帧后，Nav::update() 将公开指令清零，保留原始帧与接收 FIFO。jelly_sentry 不额外保存过洞标志，超时后只按清零的导航量处理。速度分量仍由接收回调直接更新。超时不清原始帧和接收 FIFO，坏帧持续到达也会超时。

## 哨兵控制链路与坐标

自主行驶由哨兵正赛身份或 `is_sentry_test()` 锁存进入。jelly_sentry 先将雷达系速度转到头部系，再始终叠加 VT13 平移，正赛同样叠加，不额外限幅；steer_sentry 叠加后按每轴 ±5 m/s 限幅。steer_sentry 任一 yaw 反馈离线时自主底盘停车，人工模式由 VT13 控制。

三个平面坐标系均为 X 前、Y 左，逆时针为正：

- B：底盘本体
- L：固连大 yaw 的雷达系，按雷达安装轴与大 yaw 对齐处理
- H：小 yaw 后的 IMU 头/枪口云台系，人工速度及 ChassisMatrix::set_speed 使用此系

- 两台哨兵均先按 -gimbal.yaw2.measure 将 L 系速度转到 H 系，再由底盘按总云台角转到 B 系，合成只按大 yaw 角旋转。雷达朝前、枪口偏左时，纯导航向前仍沿雷达朝向，遥控向前沿枪口朝向。
- 人工平移输入采用 H 系，导航与遥控在 H 系相加；下行 yaw 字段当前只保存。

| 过洞标志 | 模式 | 旋转策略 |
| --- | --- | --- |
| 1 | FOLLOW_MODE | 导航 wr 输入为 0，由跟随 PID 根据总云台相对底盘角对准 IMU 头，平移照常 |
| 0 | DETACH_MODE | jelly_sentry 固定 90 rpm；steer_sentry 合速度0→5 m/s对应120→60 rpm，仅插值输入限幅 |

jelly_sentry 导航断联后按清零的导航量继续叠加遥控，固定 90 rpm，云台继续扫描或自瞄。steer_sentry 导航断联后按清零的导航量继续叠加遥控。

## pitch1 固定失能

- `GimbalFold::set_enable()` 对 pitch1 固定调用 `set_enable(false)` 并写零原生输出
- `GimbalFold::update()` 更新 pitch1 反馈、清空其两级 PID，目标跟踪实测，最终 `set_output(0)`；总使能入口固定关闭 pitch1，不计算位置环和重力补偿输出
- 主应用保留默认 4 ms 的 `pitch1.send()`；DmMotor 在失能分支发送 `FF FF FF FF FF FF FF FD`，CAN ID 0x10，输出目标为零
- 导航 flag 和总云台使能均不会启用 pitch1；其他三轴保持现有控制
- 软件检查与主机测试覆盖失能状态、零电流/力矩目标和失能帧。实机是否已运行新固件及物理输出状态由用户更新固件后核对

协议验证入口见 `agents/05-comm/nav/README.md`；本轮合并结果见 `agents/06-sentry/upstream/README.md`。
