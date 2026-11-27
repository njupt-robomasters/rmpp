# GimbalFold 三轴 IMU 闭环

状态：主机仿真通过，待上车校核符号与增益

## 改动

`lib/robot/gimbal_fold.*` 去掉 ECD/IMU 双模式与窗口模式，三轴全部闭环在 IMU 世界系。`update()` 改为接收云台四元数和机体系角速度，与 `GimbalClassic` 一致。

头部世界 yaw 满足 `ψ = c + θ1 + θ2 + o`（底盘朝向 + 大小 yaw 编码器 + 安装偏角），因此：

- yaw1 自身世界角 = `ψ − θ2`，世界角速度 = `ω − yaw2.speed`
- 不需要 `set_chassis_wr()`：底盘转动直接出现在 IMU 反馈里

## 环路

| 轴 | 外环 | 内环 | 输出 |
| --- | --- | --- | --- |
| 小 yaw（主执行器） | 头部角度误差 `ψ_ref − ψ` → 世界角速度 | 世界角速度误差 `ω_ref − ω` | A |
| 大 yaw（辅助轴） | 自身世界角误差 `(ψ_ref − θ2_ref) − (ψ − θ2)` → 世界角速度 | `ω_ref − (ω − yaw2.speed)` | Nm |
| pitch2 | 世界角度误差 `p_ref − p` → 世界角速度 | `ω_ref − gyro.y` | Nm，叠加重力补偿 |

- 世界 yaw 角速度取 `(q · gyro).z`，pitch 速率取 `gyro.y`；不用 `GimbalClassic` 的 pitch 关节近似
- 小 yaw 越限时截断朝外输出，大 yaw 的世界角误差随之变大自动补足
- 不可控或恢复首帧：目标跟随实测、清空 PID、输出零；yaw 轴要求大小 yaw 同时在线，pitch 轴只要求 pitch2
- 角度外环可选加速度规划（`yaw_max_accel`、`pitch_max_accel`，默认 `INFINITY` 关闭）
- 只调 yaw2 速度环：yaw1 三个 PID 全给 0、yaw2 外环 kp 给 0，此时 `ω_ref = yaw_speed`，系统只剩 IMU 角速度环

## 验证

主机仿真 `gimbal_fold_test.cpp`：电机与刚体由一阶模型驱动，`ψ` 由 `θ1 + θ2 + 底盘角` 合成，陀螺按 Euler 关系构造。

```sh
g++ -std=c++23 -O2 -Wall -Wextra -Werror -DEIGEN_DONT_VECTORIZE -DEIGEN_MAX_ALIGN_BYTES=0 \
    -Ilib -isystem lib/third_party/eigen-3.4.0 \
    agents/06-sentry/gimbal_imu/gimbal_fold_test.cpp \
    lib/robot/gimbal_fold.cpp lib/pid/pid.cpp lib/motor/motor.cpp \
    -o agents/tmp/gimbal_imu/gimbal_fold_test.exe && agents/tmp/gimbal_imu/gimbal_fold_test.exe
```

覆盖：世界角跟踪（yaw 0.8 rad、pitch 0.2 rad 收敛，小 yaw 回中）、底盘 3 rad/s 匀速转动时头部世界角最大偏差 0.0117 rad、1 rad/s 速度指令、大 yaw 慢速时小 yaw 越限 40° 后截断并回中、仅 yaw2 速度环的符号与衰减、单 yaw 断联输出零且恢复后重新闭环、pitch 编码器上限 0.314 rad 钳制。当前全部 PASS。

`jelly_sentry` 预设的 `rmpp_app`、`rmpp_lib` 对象编译通过无警告；7 个改动手写文件通过 `tools/format_code.py --check`（clang-format 22.1.0）。未完整链接、未烧录、未上车。

## 上车待办

- 手推头部逆时针，确认 `(q · gyro).z` 为正，`attitude.yaw` 随之增大
- 确认抬枪口时 `gyro.y` 与 `attitude.pitch` 同向
- 大小 yaw 编码器合成角与 IMU 航向同向（此前已按大 yaw `is_invert=true` 翻转）
- 低输出先验 yaw2 速度环，再开 yaw2 外环，最后开 yaw1 两环；小 yaw 中位期望为 0
