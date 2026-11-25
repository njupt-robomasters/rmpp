# 轮腿力矩分配

2026-11-24 用户要求按 `E:\rm\code\potential\wheel_leg_lab` 当前仿真策略更新下板 `ChassisLeg`，本次不改 LQR 权重。提交前已把工作区现有改动提交为 `5510c9c`。

## 用户已确认

- 策略取仿真当前默认（提交 `33e2b56`）：直行目标按轮子额定转速限速，四路力矩超限时姿态组优先、运动组共同退让
- 双轮净机械功率控制暂不应用：用户实测勾选后不稳定，本次不移植
- 直行目标上限放 `Config`，本次取 40 rad/s（2.4 m/s），上车后按实车额定转速标定
- 删除航向、腿角、俯仰三项误差限幅：仿真在分配开启时不再使用 `yaw_cap`、`leg_cap`、`pitch_cap`
- 不更新 `lqr_fit.hpp`；电压包络需要的 Ke、线间电阻没有实车标定，物理端沿用 `Motor::set_torque()` 按 `max_torque` 截断

## 实现

- `Config`：新增 `wheel_work_speed`（rad/s），删除 `x_err_max`、`yaw_err_max`、`theta_err_max`、`pitch_err_max`；`x_err_max` 原本已被 `x_max` 的积分限幅覆盖，从未生效
- `calc_lqr()`：先算 `travel_speed = wheel_work_speed × wheel_radius` 和 `rim_target = |vx| + |wr| × 轮距/2`，超限时两轴目标乘同一比例；十项贡献按状态 0–3（运动）和 4–9（姿态）分两组；四路都在额定区间内时原样下发，否则求最大共同运动比例，无解时运动归零再整体缩小姿态组
- 四路额定区间：轮取 `motor.config.max_torque`；腿摆取 `2 × (joint_max − |T1|)`，`T1` 为当拍轴向力在两关节上形成的反向力矩，与 `map_force()` 共用 `calc_axial_torque()`
- `maximum_share()`：四路区间换算为比例区间求交，比例限 [0, 1]，无解返回 −1
- 跳跃期间关节余量被伸腿力占用，跳过分配，保持原输出由电机限幅
- `app/leg/leg_down/chassis.hpp` 增加 `.wheel_work_speed = 40.0f`

## 验证

- `cmake --build build/leg-leg_down`：编译链接通过，FLASH 98664 B
- 主机测试 `agents/22-leg_torque_alloc/alloc_test.cpp`：用真实 `ChassisLeg`、`M3508`、`DM8009` 与 `LQR_FIT`，22 项检查通过
  - 小旋转目标：四路原样下发，关节力矩与 `T1 ± Tt/2` 一致
  - 直行 5 m/s：目标先按 2.4 m/s 缩放，轮端 −4.736 N·m 与缩放后误差一致
  - 原地 32 rad/s：运动保留 0.521，轮端 −5.090 N·m 到限，腿摆由 −46.884 退到同比值，两关节 −15.110 / −9.314 N·m，未超过 40 N·m

## 上车核对

1. 低速直行、原地旋转和小陀螺：四路未超限时应与改动前手感一致；超限时车仍能把姿态收回来
2. 观察 `leg[].Tt`、`leg[].Ta/Te` 与电机下发差：分配后的关节力矩应始终在 ±40 N·m 内，轮端在 ±5.09 N·m 内
3. 腿承重较大时核对摆动力矩上限：`T1` 接近 40 N·m 时摆动力矩被压到零，此时只剩轴向支撑，需要确认不会因此失去腿角控制
4. 跳跃：起跳、收腿、落地缓冲期间不做分配，行为与改动前相同
5. `wheel_work_speed = 40` 对应 2.4 m/s，按实车最高稳定速度调整；导航和遥控的目标若超过该值会先被等比缩小
