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
