# 04 轮腿

状态：控制、上下板、LQR、力矩与功率分配已并入 master，待用户审计与上车

## 任务记录

| 目录 | 内容 |
| --- | --- |
| `chassis/` | 从 SOLDIER_LEG_4 迁移轮腿底盘；IMU 极性核查、电机映射、虚拟 IMU 方案 |
| `lqr/` | Python 复现轮腿 LQR；重心偏移稳态与漂移、LQI 对比；生成 `chassis_leg_lqr.hpp` 的拟合表 |
| `link_ui/` | 上下板 UI 槽位、下板状态帧与 6 个电机状态、使能与自救授权 |
| `torque_alloc/` | 额定力矩分配：姿态组优先、运动组共同退让、腿摆余量折算 |
| `power_alloc/` | 当前的双轮净机械功率闭式分配、稳姿态增益拟合表、板间功率目标 |
| `spin_power/` | 小陀螺功率限幅历史实验，限幅逻辑已被 `power_alloc` 取代，遗留结论仍适用 |

## 关键接口

- 上下板：`app/leg/leg_protocol.hpp`。上板控制帧 8 B，500 Hz（使能、批准自救、跳跃、速度、yaw、腿长、功率目标）；下板状态帧 4 B，250 Hz（倒地、实测航向角速度、6 个电机状态）。使能由下板直接透传；自救需上板倒地时按 Pause 或 V 批准一次，站起后重新批准
- 下板 `ChassisLeg::calc_lqr()` 先按额定力矩分配，再按 `power_ref` 修正；FSi6X 在线时优先接管，上板断联 0.1 s 后失能
- LQR 与功率映射表在 `lib/robot/chassis_leg_lqr.hpp`，由 `lqr/gen_lqr_fit.py` 生成，不手改
- 功率口径只约束两轮机械功率 `Σ Tw·ω`，关节保持力矩与铜耗不计；仿真对照见 `power_alloc/README.md`

## 文档

- `../02-conventions/application.md` 的“轮腿上下板通信”
- 根 `docs/chassis-leg.md`：轮腿控制思路，面向队员

## 待办

功率口径对照、航向前馈方向、自瞄链路、裁判与 UI、首版 LQR 参数对照、限功率时的平衡稳定性，详见 `../TODO.md`
