# MiniPC 通信与导航

## 通道与接口

- mc02 `app/jelly_sentry`：`Aim(Cdc(1), {.timeout = 200 * ms})` 使用第一路双向通信，`Nav(Cdc(2), {})` 独立使用第二路
- C 板 `app/sentry` 只有一路 CDC，Nav 调用同步迁移为 `Nav(Cdc(1), {})`，与 Aim 配置共用端口；其底盘保留原有导航小陀螺和人工叠加策略，本轮的 flag 模式仲裁位于 jelly_sentry
- Nav 构造注册 USB 接收回调；对象在回调使用期间持续存活，现有应用均为全局实例
- `nav.update()` 在主循环的 `chassis_loop()` 前调用，公开只读输出为 `is_connect` 和 `command.{vx,vy,yaw,action_flag}`
- 已移除旧 `chassis_speed`、`target_position` 接口及两处应用只在本地赋值的 GO_CENTER/GO_HOME 占位流程；目标点与返航由上位机规划，目标点/状态上行协议待约定

## 导航下行协议

固定 17 字节，小端 IEEE 754 float32：

| 偏移 | 字节数 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 2 | SOF | A5 5A |
| 2 | 4 | vx | 最终 IMU 头坐标系前向速度，m/s |
| 6 | 4 | vy | 最终 IMU 头坐标系左向速度，m/s |
| 10 | 4 | yaw | 上位机角度，rad；当前解析和保存，具体控制用途待约定 |
| 14 | 1 | action_flag | 0 允许小陀螺，1 随头；当前 pitch1 全程失能 |
| 15 | 2 | CRC16 | MODBUS，低字节在前 |

CRC 初值 0xFFFF，反射多项式 0xA001，无结果异或，覆盖偏移 2–14 共 13 字节。独立于裁判系统的 CRC16。参考帧（vx=1、vy=0、yaw=0、flag=0）：

```text
A5 5A 00 00 80 3F 00 00 00 00 00 00 00 00 00 9A E6
```

回调逐字节寻找帧头，收齐后检查 CRC、三个浮点数的有限性及 flag 的 0/1 范围。坏帧滑动一个字节重新同步；支持分片、粘包、噪声与重叠帧头。有效帧覆盖待发布指令，`update()` 用短 `IrqLock` 发布完整快照，主循环读取期间保持一致。

默认 100 ms 无有效帧则离线，公开 command 清零，坏帧持续到达也会超时。超时清除旧连接半帧，新连接首帧允许任意分片。应用依据 `is_connect` 判断有效性，离线时清零后的 flag=0 不会启动小陀螺。

## MCU → 导航：大 yaw 实测角

`jelly_sentry` 调用 `nav.update(gimbal.yaw1.measure, yaw1.is_connect)`，由 Nav 内部判断大 yaw 在线并通过 CDC2 每 5 ms 上报一次，固定 8 字节。无参数 `nav.update()` 维持仅接收行为：

| 偏移 | 字节数 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 2 | SOF | A5 5A |
| 2 | 4 | yaw | 大 yaw 相对底盘的实测角，float32 小端，rad，范围 [-π, π)，左转为正 |
| 6 | 2 | CRC16 | MODBUS，小端，覆盖偏移 2–5 共 4 字节 |

角度已应用电机零位、方向和减速比；电机离线时暂停上报，CDC 忙时沿用 BSP 丢弃策略。yaw=1 rad 的参考帧：

```text
A5 5A 00 00 80 3F 21 F4
```

## jelly_sentry 控制链路与坐标

导航在整车使能、自主模式、Nav 在线且大小 yaw 均在线时生效。人工底盘输入优先接管速度，人工平移保持有效 flag 对应的小陀螺或随头模式；人工主动旋转优先于自动旋转，松开且键盘速度归零后恢复有效导航。Fn 仅启用自主模式。Nav 或 yaw 反馈离线时停止自动平移、自动旋转和随头，模式为 DETACH_MODE，人工仍可接管。

人工和导航平移统一使用最终 IMU 头坐标系，X 前、Y 左，逆时针为正：

- B：底盘本体
- H：小 yaw 后的 IMU 头/枪口云台系，人工、导航速度及 ChassisMatrix::set_speed 使用此系

令 q1 为大 yaw 相对底盘角，q2 为小 yaw 相对大 yaw 角，总角 q=q1+q2：

```text
上位机直接下发 H 系速度 v_H
底盘内部 H 系 → B 系：v_B = R(q) v_H
```

总角取 `gimbal.yaw.ecd.measure`。导航接入处直接使用 vx/vy，坐标旋转由底盘内部按总云台角完成。

| 有效 flag | 模式 | 旋转策略 |
| --- | --- | --- |
| 1 | FOLLOW_MODE | 关闭小陀螺，无人工旋转输入时由跟随 PID 对准 IMU 头，人工平移保持随头 |
| 0 | DETACH_MODE | 关闭随头，无人工旋转输入时以 60 rpm 自动小陀螺旋转，人工平移保持小陀螺 |

模式随最新有效 flag 持续设置。平移每轴限幅 ±5 m/s，人工主动旋转限幅 ±100 rpm，自动小陀螺为 60 rpm，随头 PID 上限保持 60 rpm，底盘继续执行原有轮电机闭环和超电功率控制。yaw 字段目前仅供协议和 SWD 观察。pitch1 的 flag=0 抬头、flag=1 折叠仅记录为 TODO，当前保持失能。

## pitch1 固定失能

- `GimbalFold::set_enable()` 对 pitch1 固定调用 `set_enable(false)` 并写零原生输出
- 每次 `GimbalFold::update()` 再固定 pitch1 失能，更新反馈、清空 pitch1 两级 PID，目标跟踪实测，最终 `set_output(0)`；pitch1 位置环和重力补偿输出计算已撤除
- 主应用保留默认 4 ms 的 `pitch1.send()`；DmMotor 在失能分支发送 `FF FF FF FF FF FF FF FD`，CAN ID 0x10，输出目标为零
- 导航 flag 和总云台使能均不会启用 pitch1。共享 GimbalFold 的云台调试目标同样遵循此策略；其他三轴保持现有控制
- 软件检查与主机测试覆盖失能状态、零电流/力矩目标和失能帧。实机是否已运行新固件及物理输出状态由用户更新固件后核对

验证脚本与本轮结果见 `../tasks/13-minipc_nav/README.md`。

## Aim 自瞄双向协议

`Aim` 保存 CDC 句柄和只读 Config。`Config::timeout` 为固定指令有效期，默认 `200 * ms`，从最后一帧有效视觉数据计时，与接收帧率独立。修改示例：`Aim aim(Cdc(1), {.timeout = 100 * ms});`。设备和 Config 均显式传入。

应用每周期执行 `set_referee(is_red, bullet_speed)`，再调用 `update(attitude.yaw, attitude.pitch, attitude.roll)`，测量来自最终安装 IMU 的头部；弹速取裁判 `shoot_data.initial_speed`（m/s），蓝方为 0、红方为 1。Aim 内部以 5 ms 周期上报，视觉结果在每次 update 中发布。

### MCU → 视觉：21 字节，200 Hz

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | A5 5A |
| 2 | 4 | 头部 yaw，float32，deg，左正 |
| 6 | 4 | 头部 pitch，float32，deg，上正 |
| 10 | 4 | 头部 roll，float32，deg，沿枪口由后向前看顺时针正 |
| 14 | 1 | is_red，0 蓝、1 红 |
| 15 | 4 | bullet_speed，float32，m/s |
| 19 | 2 | 小端 MODBUS CRC16，覆盖偏移 2–18 共 17 字节 |

上报持续进行，CDC 忙时沿用 BSP 的丢弃策略，下一周期发送最新数据。姿态非有限值时跳过该次上报，非法弹速按 0 上报。

### 视觉 → MCU：18 字节，随新结果发送

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | A5 5B |
| 2 | 1 | is_detect，0/1 |
| 3 | 4 | 最终 IMU 头期望 yaw，float32，deg，左正 |
| 7 | 4 | 最终 IMU 头期望 pitch，float32，deg，上正 |
| 11 | 1 | is_fire，0/1 |
| 12 | 4 | distance，float32，m，非负 |
| 16 | 2 | 小端 MODBUS CRC16，覆盖偏移 2–15 共 14 字节 |

全部多字节字段为小端，浮点为 IEEE 754 float32。Aim 和 Nav 共用 `lib/minipc/protocol.hpp` 的 CRC 与浮点编解码；裁判 CRC 保持独立。接收支持拆包、粘包、噪声与滑动重同步。CRC、标志范围、有限浮点和非负距离全部通过才刷新计时；中断写 pending，主循环用短临界区取得完整快照。

有效的 `is_detect=0` 帧保持通信在线并立即清空目标和开火许可。超过 Config::timeout 后，is_connect_vision=false，vision 全部恢复默认，is_detect/is_fire=false；持续坏帧也会超时。接收时间作为指令新鲜度依据，上位机重发旧结果时需自行管理结果年龄。

### 最终 IMU 头的角度语义

- 目标为与上传 IMU 姿态相同世界参考系的头部绝对 yaw/pitch，包含视觉提前量与弹道补偿
- 项目内部采用右手 ZYX 弧度：yaw 绕 Z 左正，pitch 绕 Y 正向为低头，roll 绕 X 正向从头部后方沿枪口看为顺时针
- 线上 yaw/roll = 内部角 / deg；线上 pitch = -内部 pitch / deg。下行相应反向换算，并由 Angle 归一化
- 现有 `gimbal_loop()` 将 `aim.vision.yaw/pitch` 传入 IMU_MODE 的 `gimbal.set_angle()`，GimbalFold 分配大小 yaw 与 pitch2；pitch1 继续固定失能，导航控制仍使用 CDC2
- 大小 yaw 分配采用 ±45° 小轴工作窗口，大轴在窗口内保持世界朝向、超限后追随至 ±35° 内，小陀螺由大轴反向补偿；目标始终指向最终 IMU 头，详见 `application.md`“jelly_sentry 大小 yaw 工作窗口”
- 原 Aim 中未用于本协议的公开 imu 写入、比赛阶段/剩余时间字段已移除；裁判输入经 set_referee 设置。全景相机占位 a0 继续保持 NAN，其协议待后续约定

本轮验证与复现见 `../tasks/14-minipc_aim/README.md`。上板核对头部 yaw/pitch 正方向和 roll 观察方向，确认视觉与 MCU 使用相同世界航向零点。
