# 折叠云台小 yaw 接入陀螺仪速度内环

状态：已在主工作区完成实现、主机验证与 ARM 对象编译，待用户审计与上车

## 改动内容

- `lib/robot/gimbal_fold.{hpp,cpp}`：参考 `GimbalClassic`，将 `GimbalFold::update` 签名更新为 `update(const Quaternionf& q, const Vector3f& gyro)`，由 `q` 解出 `yaw.imu.measure` 与 `pitch.imu.measure`
- 小 yaw（`yaw2`）轴向速度反馈：`imu.gyro` 经 `pitch2` 关节角 `AngleAxisf(pitch.ecd.measure, Vector3f::UnitY())` 旋转到小 yaw 随动系，取 `Z` 分量作为小 yaw 的绝对角速度内环反馈，不再使用小 yaw 电机编码器差分速度
- 小 yaw 外环修正与速度前馈：
  - 未触及 `±50°` 限位时，`IMU_MODE` 下将世界偏航速率修正经 `pitch_joint * (q.conjugate() * Vector3f(0, 0, yaw2_correction))` 投影到电机轴，速度内环直接跟踪云台目标角速度 `yaw_speed`（底盘旋转与大 yaw 运动由 `1 kHz` 陀螺仪内环直接抵消，移除旧 `-chassis_wr - motor.yaw1.speed` 相对速度前馈）
  - 触及 `±50°` 限位时，保持关节角误差修正并叠加 `chassis_wr + motor.yaw1.speed`，使小轴贴限位平滑随大轴转动
- `pitch2`、大 yaw（`yaw1`）与 `pitch1`：
  - `pitch2` 完整恢复原有编码器速度闭环（`motor.pitch2.speed`）、限位判断、重力补偿与 `app/jelly_sentry/gimbal.hpp` 中的全部原 PID 参数配置，不做任何改动
  - `yaw1` 保持原有编码器速度内环（`motor.yaw1.speed`）与 `-chassis_wr` 前馈，暂不加入平方根减速规划与小轴反作用力矩前馈；`pitch1` 继续保持全程固定失能与零输出
- `app/jelly_sentry/gimbal.cpp`：`gimbal_loop()` 调用 `gimbal.update(attitude.q, imu.gyro)`，`app/jelly_sentry/gimbal.hpp` 保持零改动

## 验证与复现

```powershell
python agents/06-sentry/yaw_gyro/verify.py
```

- `gimbal_fold_gyro_test.cpp`（1,112 项检查）：覆盖 `pitch2` 关节角旋转下的陀螺仪投影、枪口自旋 `-sin(theta)` 投影、小 yaw 编码器速度噪声隔离、`pitch2` 与大 yaw 原编码器速度内环保持不变、小陀螺及大轴超调扰动下的世界系零稳态误差与即时阻尼、越限 `±50°` 随动大轴、倾斜底盘世界偏航收敛、单电机掉线隔离及 `pitch1` 零输出
- 局部编译：`lib/robot/gimbal_fold.cpp` 与 `app/jelly_sentry/gimbal.cpp` 通过 ARM GCC 对象编译，零警告；`clang-format` 与 `git diff --check` 通过

## 上车核对

- 低输出限幅下手动轻推小 yaw，确认小 yaw 陀螺仪内环阻力方向反向、无正反馈自激
- 方向确认无误后，可在实车整定时逐步调高或放开 `gimbal.hpp` 中 `yaw2_speed_pid.err_fc`（当前保留原实车参数 `10 Hz` 未动），释放小 yaw 内环相位裕度
