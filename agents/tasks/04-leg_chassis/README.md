# 04 轮腿底盘迁移

## 目标

把 `E:/rm/code/potential/SOLDIER_LEG_4/Chassis`（下称外部工程，已实车可用）的轮腿底盘控制迁移到本工程；姿态来源由外部传入，可为本板 IMU，也可为云台 IMU + 云台关节角换算的虚拟 IMU（本轮只接通用姿态）

状态：分析完成，未改代码，待用户确认（见末尾）

## 坐标约定（迁移后统一）

- 底盘机体系 FLU：X 前、Y 左、Z 上；q 为机体到世界
- `pitch` 绕 Y：低头为正；`roll` 绕 X：右侧低为正；`yaw` 绕 Z：俯视逆时针为正
- 角速度为机体系 `gyro`，`gyro.y` 低头为正，`gyro.x` 右侧下沉为正，`gyro.z` 逆时针为正

## 外部工程 IMU 核查结论

1. EKF 与本工程同源（Wang Hongxi），q 为机体到世界，`q̇ = ½ q⊗ω`
2. 欧拉角命名错位：`QEKF_INS.Pitch = atan2(2(wx+yz), 1-2(x²+y²))` 实为绕 X（标准 roll）；`QEKF_INS.Roll = asin(2(wy-xz))` 实为绕 Y（标准 pitch）；`Yaw` 正确
3. `IMU_Param` 安装角全 0，修正矩阵为单位阵，外部机体系即 C 板 BMI088 原始轴
4. 底盘换算：`imu_pitch = -INS.Roll`、`imu_d_pitch = -Gyro[1]`、`imu_roll = INS.Pitch`、`imu_d_roll = Gyro[0]`、`imu_d_yaw = Gyro[2]`
5. 由两条实车可用的闭环反推传感器轴：
   - 腿倾角 `theta = rel_theta + imu_pitch`（后摆为正）要求 `imu_pitch` 低头为正 ⇒ 传感器 +Y 指向车右
   - 横滚补偿 `roll_pid(0 - imu_roll)`，左腿目标 `L - out`、右腿 `L + out`：roll>0 时左腿伸长 ⇒ `imu_roll>0` 为左侧低 ⇒ 传感器 +X 指向车后
   - 右手系 ⇒ 传感器 +Z 向上，与 `imu_d_yaw` 和运动学 `kin_d_yaw`（右轮快为正 = 逆时针）同号一致
   - 即外部 C 板相对 FLU 安装为 yaw 180°
6. 恒等式 `Rz(ψ)Ry(θ)Rx(φ)Rz(π) = Rz(ψ+π)Ry(-θ)Rx(-φ)` 对任意姿态精确成立，所以外部控制量与标准 FLU 量的关系：

| 外部变量 | 标准 FLU 量 | 关系 |
| --- | --- | --- |
| `imu_pitch` | `pitch` | 相同（低头正） |
| `imu_d_pitch` | `gyro.y` | 相同 |
| `imu_roll` | `roll` | **相反**（外部左侧低为正） |
| `imu_d_roll` | `gyro.x` | **相反** |
| `imu_yaw` | `yaw` | 差 π，只存未用 |
| `imu_d_yaw` | `gyro.z` | 相同 |

结论：迁移后只有横滚取反，横滚补偿的腿长分配须同步改号；俯仰、航向不变

本工程配置：若板卡安装与外部车一致，`Imu::Config::orientation = euler_to_quaternion({.yaw = 180 * deg})`

## 其他极性映射

- 遥控：外部 `RC_Data.x = ch2`、`y = -ch1`（1 起始），与本工程 `FSi6X::x/y` 完全相同；`d_yaw = y·max`，右摇杆左推为逆时针；拨杆 UP/DOWN 物理位置一致
- 关节 DM8009（CAN2，tx 0x11–0x14 / rx 0x21–0x24）：外部 `pos = wrap(raw + c)`，c = 2.754 / 0.798 / 1.115 / -1.259；`phi4 = 0 - pos`、`phi1 = π - pos`，`d_phi = -vel`；力矩 `T_A/T_E` 以电机原始方向直接下发。本工程 `position = dir·(raw - offset)`：`is_invert = false`、`offset = -c` 时 `position` 与外部 `pos` 相同，`set_torque` 与外部同向
- 轮 M3508（CAN2 0x201 左、0x202 右）：外部左轮速度与电流命令都取反、右轮不取反 ⇒ 左 `is_invert = true`、右 `false`；`reduction = 16.875`、`kt = 0.2545`（轮端 N·m/A），`v_wheel = speed · WHEEL_RADIUS` 前进为正
- 云台相对航向：`err[2] = gimbal_yaw - cmd.gimbal_yaw`，即底盘航向“目标减反馈”（逆时针正）；外部两者都是常量 `YAW_FRONT`，该项恒为 0，跟随从未实车验证

## 外部工程注释与实现不符（迁移时以代码为准）

- `Can_Feedback.c` 布局图写 CAN1 0x10/0x11，代码全部在 CAN2 0x21–0x24，且左右 A/E 对应与 `State_Update` 不一致
- `kin_x` 注释“限幅 ±0.25 m”，代码为 0.1 m
- `roll_pid` 使用 10 阶最小二乘微分（OLS），本工程 `Pid` 无此功能
- 外部 PID 用 `abs` 宏且死区 0：误差恰为 0 时不更新输出
- `Off_Detection`、`Slip_Detection`、`Bump_Control` 未调用

## 虚拟 IMU 方案（本轮不实现）

底盘只接收“底盘机体 q + 机体系 gyro”。云台关节相对底盘 `R = Rz(ψm)·Ry(θm)`（ψm 云台相对底盘逆时针正，θm 低头正；IMU 在 yaw 级时 θm 取 0）：

- `q_chassis = q_gimbal ⊗ R⁻¹`
- `gyro_chassis = R·gyro_gimbal - ψ̇m·ẑ - Rz(ψm)·θ̇m·ŷ`

风险：关节零位误差直接变成底盘 pitch/roll 偏差；GM6020 速度分辨率 1 rpm ≈ 0.105 rad/s，直接相减会严重劣化 LQR 的 `d_pitch`，需改用编码器差分或滤波；云台运动扰动 EKF 重力观测；跨板传输延迟。底盘平衡建议优先用本板 IMU
