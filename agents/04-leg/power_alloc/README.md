# 轮腿功率分配

状态：已并入 master，待用户审计与上车

2026-11-25 用户要求按 `E:\rm\code\potential\wheel_leg_lab` 当前仿真（提交 `9649a28`，drive-27 无倍率闭式分配，见仿真仓库 `wheel_leg_lab/agents/live_drive/power_80w.md`）适配下板 `ChassisLeg`，目标是功率受限时的姿态稳定性。上一轮额定力矩分配见 `agents/04-leg/torque_alloc/`。

## 用户已确认

- 功率预算经板间协议下发：恢复 `f28c62f` 删除的 `power_ref` 字段（11 位、0.25 W/LSB、2047 不限），上板取 `super_cap.power_chassis_ref`
- 两腿稳姿态增益离线表追加到 `lib/robot/chassis_leg_lqr.hpp`，与 `LQR_FIT` 同一生成脚本、同一 Q/R；`M`、`H⁻¹` 在控制拍内由实时增益求 2×2

## 实现

- `agents/04-leg/lqr/gen_lqr_fit.py`：新增 `LQR_POSTURE_FIT[2][6]`（锁定轮力矩，六状态子系统按原 Q/R 子块求 LQR，取两腿对俯仰角速度的增益，poly22 拟合，最大相对误差 5.80%、绝对 1.44 N·m/(rad/s)）与 `LQR_R[4]`；重新生成后 `LQR_FIT` 数值不变
- `ChassisLeg::calc_lqr()`：额定力矩分配后计算 `wheel_power = ω·Tw`；超过 `power_ref` 时按仿真 `limit_wheel_power()` 修正，跳跃期间不修正
- `set_power_ref()`：W，`INFINITY` 不限；leg_down 上板控制时取协议值，FSi6X 控制时不限
- `lib/math/eigen.hpp` 增加 `Matrix2f` 别名

## 验证

- `python agents/04-leg/power_alloc/check_maps.py`：固件口径（拟合表求 `M`、`H⁻¹`）与仿真 `power_maps()` 对照；0.18 m 时 `M` 最大差 0.006（量级 9.6），0.30 m 时 0.23（量级 15.3），`H⁻¹` 相对差 < 0.2%；`det(C)` 从 0.14 m 的 −4.49 到 0.30 m 的 −1.33，不接近奇异
- `python agents/04-leg/power_alloc/check_power.py`：编译真实 `ChassisLeg` 跑 7 个工况，47 项自检通过；与仿真 `limit_wheel_power()` 用同一映射复算最大差 1.7e-5 N·m，换用仿真精确映射最大差 0.025 N·m
  - 直行巡航（两轮 40 rad/s、预算 80 W）：93.19 → 80.00 W，两轮 1.165 → 1.000 N·m，两腿 11.59 → 9.23 N·m
  - 轮端顶边（60/2 rad/s、预算 20 W）：投影越界后截取，右轮停在 5.09 N·m，功率恰为 20 W
  - 制动负功、预算充足、单轮反转净功率为负：原样下发
- `python agents/04-leg/chassis/test_leg_up_control.py`：协议位布局、功率编码（含 `INFINITY` → 2047、限幅、上板取超电目标）通过
- ARM 构建 `build/leg-leg_down`（FLASH 101600 B）与 `build/leg-leg_up`（FLASH 114260 B）通过，无告警
- 以上均为主机测试和编译；未上车，未测 MCU 执行时间

## 上车核对

1. 功率口径：仿真与固件都只约束两轮机械功率 `Σ Tw·ω`，超电目标是整车底盘电功率，关节保持力矩与轮电机铜耗不计。对照超电 `power_chassis` 与下板 `wheel_power`，若长期超出裁判上限，再决定是否在上板给协议值留余量
2. 满速直行与边走边转：观察 `wheel_power` 超过 `power_ref` 时车身俯仰和腿角；仿真 0.18 m 满 W 末秒俯仰 7.85°
3. 仿真里实验权重满速直接反向和高腿 W+A 反向仍有越界，实车先在低腿长、缓加速下试
4. 上板断联时协议数据清零，`power_ref` 为 0，同时底盘失能，不影响停车
