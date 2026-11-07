# 03 四舵轮底盘适配

## 目标

把参考仓库 Joushua233/rmpp（38acb4d）`rmpp/lib/module/chassis/Chassis_DualSteer.*` 的四舵实现移植到新框架，并按“控制方式 × 底盘构型”重构底盘库；底盘、云台、发射库统一处理电机掉线

参考代码已在实车验证可用，行为以它为准；代码习惯和风格以新框架为准。参考仓库只读克隆在 `E:/rm/code/potential/rmpp-ref/joushua`，不入库

在 `quad_steer` 分支实施（基线 44e8f33），已合并到 master 6682d41

## 已确认

- 所有底盘采用“控制方式 × 构型”拆分；舵轮提供 `ChassisSteerSpeed` / `ChassisSteerTorque`
- 沿用参考仓库 125 Hz 回传和 40 ms 超时，不动 CAN 分配
- GM6020 保留电压帧，反电动势前馈系数在电机定义时给出，Ke 默认取手册 13.33 rpm/V，可覆盖
- 功率估计电阻放底盘 config，填线间电阻；iq 按 VESC 等幅值变换（M3508 最大 20 A），铜耗 0.75·R线·iq²；不算舵电机
- 掉线：目标跟随实测、清 PID、输出零，重连第一周期只初始化
  - 舵电机离线对应轮停转，轮离线舵继续转向
  - 云台 yaw、pitch 互不影响；大小 yaw 任一在线时在线一侧单独保持指向
  - 发射机构各电机不耦合，便于调试

## 结果

底盘库 `lib/robot/chassis/`：

| 文件 | 内容 |
| --- | --- |
| `chassis` | 公共：模式与跟随、云台系换算、轮电机在线管理、功率估计 |
| `chassis_torque` | 力控：vxyz PID → 牵引力 → 按力功率控制 → 构型力学逆解 |
| `chassis_speed` | 速度控：构型速度逆解 → 四路轮速 PID → 电流二次式功率衰减 |
| `chassis_omni` / `chassis_mecanum` | `ChassisOmniTorque`、`ChassisMecanumTorque` |
| `chassis_steer` | `Steer`（单舵向）、`ChassisSteerSpeed`、`ChassisSteerTorque` |

- sentry 用 `ChassisSteerSpeed`（与参考实车行为一致），infantry/示例用 `ChassisOmniTorque`
- 舵向 PID 由参考 7 V/40°、限幅 25 V 换算为 0.84 A/40°、限幅 3 A（电压帧 3 A ↔ 25 V）
- sentry 发送增加 w3/w4（0x200）与 s3/s4（0x2FF），yaw2 加完整反电动势前馈，舵电机 0.5
- 云台：`check_yaw/check_pitch` 统一离线处理；发射：每个摩擦轮独立 PID（原共用一个会串积分）

## 验证

- 已格式化改动文件；合并后 sentry、infantry、leg、examples-chassis/gimbal/shooter 编译通过，无告警
- 待上车：架空检查四舵零位与转向、平移/旋转/小陀螺方向、拔单个舵电机时对应轮停转、功率估计与裁判系统 `chassis_power` 对比（偏差大时补静态功耗或拟合铜耗系数）
- infantry 全向轮功率铜耗系数由 3·R/2 变为 0.75·R，力控不使用铜耗，仅影响 `power.estimate` 显示

## 第二轮：矩阵底盘、统一功率控制、PID 回算、UI 下放

### 结果

| 文件 | 内容 |
| --- | --- |
| `chassis_matrix` | `ChassisMatrix` 公共部分，`ChassisMatrixSpeed`、`ChassisMatrixTorque`；常量 `OMNI`、`MECANUM`（J，4×3） |
| `chassis_steer` | `Steer`、`ChassisSteer` 公共部分，`ChassisSteerSpeed`、`ChassisSteerTorque`；不继承 `ChassisMatrix`，接口同名 |
| `chassis_power` | `ChassisPower`：功率估计、超电、两组电流衰减求解；底盘公开成员 `power` |

- 矩阵正逆解为纯函数：`forward/backward`（速度 J⁺/J）、`forward_force/backward_force`（Jᵀ/(J⁺)ᵀ）；J⁺ 构造时由 (JᵀJ)⁻¹Jᵀ 求得
- 舵轮 `forward/forward_force` 为公开 const；`backward` 含舵向闭环，为 protected 非 const
- 功率：各轮电流 I = gain_xy·I_xy + gain_z·I_z，预测 Σ(0.75·R·I² + kt·I·ω) + error；缓冲能量折算上限 max·clamp((E−30)/30)
  - 力控：I_z 由 fz 单独逆解；矩阵速度控：F_z = (JᵀI)_z，I_z = (J⁺)ᵀ[0,0,F_z]；舵轮速度控整体衰减
  - |wr| > reserve_wr 先解 gain_z（gain_xy = 1），gain_z = 0 仍超时解 gain_xy；否则同值
- `Pid::track(actual)`：积分扣除 actual − out 且只向零收缩不过零，out = actual；`calculate` 的 max_out 限幅也走 track，删除原条件积分
  - 接入：矩阵/舵轮轮速 PID、矩阵/舵轮 vxyz PID（实际 torque_ref 经力正解）、舵向 PID、云台速度环、双 yaw（取被截更多的在线一侧）、摩擦轮、拨弹、IMU 加热
  - 不接入：跟随 PID、云台位置环（输出为目标）、轮腿（PD）
- UI：chassis/gimbal/shooter 删除 `update_ui`；infantry/sentry `app.cpp` 直接读电机在线；shooter 公开 `rub_speed[]`（m/s）
- ChassisMatrixTorque/ChassisSteerTorque 的 `fx/fy/fz.ref` 改为功率与限幅后的实际输出
- 删除：`Chassis`、`ChassisSpeed`、`ChassisTorque`、`ChassisOmni*`、`ChassisMecanumTorque`（由矩阵类取代），底盘 `set_power_limit/set_buffer_energy`（改 `power.set_limit/set_buffer_energy`）

### 验证

- clang-format（CLion 同版本）：`pid`、`chassis_matrix`、`chassis_power`、`chassis_steer`
- dummy、infantry、sentry、leg、examples-chassis/gimbal/shooter 完整构建通过，无告警
- 顺带迁移 `Motor::Config::Install` 字段顺序（is_invert 在 offset 前）的 12 处指定初始化，数值不变
- 复现：PATH 加 CLion cmake/ninja 与 Arm GNU，`cmake --preset <p> && cmake --build build/<p>`

### 上车核对

- 力控开始计入铜耗：infantry 未配 `power.wheel_resistance`，铜耗为零，行为与原来一致；配置后起步会更保守
- 速度控（dummy）新增旋转优先：小陀螺 + 平移限功率时确认先降转速
- 缓冲能量曲线改为折算上限，力控删除 45 J 特例
- 回算后限功率解除不再冲出积分尖峰；云台、摩擦轮饱和后的响应手感需复核
- 云台速度环 `fc` 低通时，饱和期间 out 被同步为限幅值

## 第三轮：Config 整理与约定推广

### 结果

- 底盘构造 `(Config, PowerLimiter&, wheel[, steer])`；`kinematics`、`speed_pid_config` 并入 Config，`reserve_wr` 改 `set_reserve_wr()`
- `ChassisPower` 改名 `PowerLimiter` 移至 `lib/power/`，由 app 持有：`super_cap` → `power_limiter` → `chassis`；裁判上限与缓冲能量用 `power_limiter.set_*` 设置
- 运动学 `forward_speed/backward_speed/forward_force/backward_force`，云台系换算用 `rotate(Vector3f, angle)`
- Motor 断联清零电流、力矩、速度；Pid 拆为 `err_fc` 与 `d_fc`
- 约定推广：Gimbal、Shooter、Aim、Nav、FSi6X、VT13 的 `config`/`is_enable`/`mode` 私有化；Imu、Melody、Referee、ChassisLeg 删除 `config()`；Motor `is_enable_` 改 protected；EKF、Melody 文件私有辅助函数改 private static；标量 `{}` 改 `= 0`
- 保留公开：`Motor::config`（功率限制、云台零点、卡弹判断读取）、`SuperCap::config()`（PowerLimiter 读裕量与补偿电流）

### 验证

- dummy、infantry、sentry、leg、examples-chassis/gimbal/shooter/attitude/melody/motor/rc/referee 编译通过；examples-motor 仅有原有未使用函数告警

### 上车核对

- PID 低通从输出移到误差输入，原 `fc = 10` 的云台、infantry 底盘手感需复核
- infantry/sentry 每周期显式设置分离/跟随，使能切换不再隐含回到分离
- 断联反馈清零影响全部机构
