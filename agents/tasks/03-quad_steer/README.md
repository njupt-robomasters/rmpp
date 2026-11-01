# 03 四舵轮底盘适配

## 目标

把参考仓库 Joushua233/rmpp（38acb4d）`rmpp/lib/module/chassis/Chassis_DualSteer.*` 的四舵实现移植到新框架：`lib/robot/chassis/chassis_dual_steer.*` 改为四舵 `ChassisSteer`，sentry 底盘随之迁移

参考代码已在实车验证可用，行为以它为准；代码习惯和风格以新框架为准

参考仓库只读克隆在工作区外 `E:/rm/code/potential/rmpp-ref/joushua`，不入库

## 已确认

- 类名 `ChassisSteer`，删除双舵
- 沿用参考仓库的 125 Hz 回传和 40 ms 超时，不动 CAN 分配
- 底盘、云台、发射机构库都应检测电机掉线并清空/设置对应变量

## 参考仓库实际控制链路

旧框架 `Motor::OnLoop` 内含软件闭环（在主控上算，不是电机固件）：

- 轮：`SPEED_MODE` + `wheel_pid`，由速度逆解的轮速闭环到电流；`backward()` 里的 `SetTorque` 会被速度环覆盖，底盘 vxyz PID 与基类基于力的功率控制实际不起作用
- 舵：`ANGLE_MODE` + `servo_pid`（单级角度环，圆周误差），输出单位 V；发送用 `GetVoltageCmd(0.5f)`，电压 = PID 输出 + 0.5·Ke·ω（反电动势补偿减半，Ke = 1 V / 13.33 rpm）
- 断联/失能：电机内 PID 清空，发送 0；底盘对舵断联禁用对应轮、清零、标记舵向重新初始化
- 功率：轮速度环之后按电流二次式整体衰减轮力矩，a=0 时用 -c/b
- sentry 未配 follow PID（跟随模式 wr=0）

## 与本地差异

| 项 | 参考仓库 | 本地 |
| --- | --- | --- |
| 轮 | 轮速环 | 底盘力控，无轮速环 |
| 舵输出 | 电压 + 0.5 反电动势补偿 | 0x1FF 电压帧按 current/3A×25000 缩放，无补偿 |
| 舵断联 | 对应轮停 | 无处理 |
| 使能后舵向 | 实测角初始化 | ref_v=0，使能瞬间转回 0 |
| 转舵未到位 | 轮速×clamp(cos 误差, 0, 1) | 无 |
| 功率 a=0 | -c/b | 除零 NaN |
| follow PID | 无 | sentry 传了 `&vxyz_pid`（量纲为力，疑似误配）|

- offset/invert 语义两边一致：`position = dir · (raw - offset)`，参考零位可直接用
- 舵 PID 换算：本地电压帧 3 A ↔ 25 V，参考 7 V/40°、限幅 25 V 对应 `set_current` 0.84 A/40°、限幅 3 A

## sentry 配置（取自参考仓库）

- 轮距 360 mm 正方形：`CHASSIS_RADIUS = 180√2 mm`，`WHEEL_RADIUS = 60 mm`，轮减速比 14
- 轮 M3508 0x201–0x204（0x200 帧）；舵 GM6020 0x207、0x208（0x1FF 帧，与 yaw2 0x205、shoot 0x206 共用）、0x209、0x20A（0x2FF 帧）
- 舵零位 -133.5° / 105.7° / -168.5°（右后含 180° 修正）/ -18.4°，舵 invert、轮不 invert
- 轮 PID 20 A/360 rpm、ki 20 A/15°、限幅 20 A
- 底盘电机 `bus.timeout = 40 ms`

## 待确认

见对话：轮控制链路、GM6020 反电动势补偿位置、掉线处理范围与行为、舵/轮 PID 配置归属、follow PID

## 验证

- 格式化改动文件，编译 sentry 目标涉及文件
- 上车：架空检查四舵零位与转向、平移/旋转/小陀螺方向、拔单个舵电机时对应轮停转、功率限制
