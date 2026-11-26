# 18 轮腿上下板协议与 UI 槽位调整

状态：已并入 master，待用户审计与上车

## 目标

- UI 不再显示裁判断连，中心弧固定绿色
- RobotUi 槽位改为 yaw1、yaw2、pitch1、pitch2；轮腿下板状态放在 yaw1
- leg_up 使能后首次 pause 才下发使能；删除自救授权，下板默认允许
- 下板状态帧回传 6 个底盘电机状态与下板使能；下板 FSi6X 在线时优先于上板
- 删除 `GimbalClassic::Config::use_angle_loop`，角度 PID 比例增益为 0 时等效只有速度内环

## 改动

- `lib/referee/robot_ui.*`：字段 `base`/`yaw`/`pitch`/`referee` 改为 `yaw1`/`yaw2`/`pitch1`/`pitch2`；新增 pitch2 图元 `0x003107`，坐标为暂定值；中心弧 `0x000160` 固定绿色
- `app/jelly_sentry/referee.cpp`：yaw1、yaw2、pitch1、pitch2 分别显示四个云台电机
- `app/steer_sentry/referee.cpp`：yaw1、yaw2、pitch1 显示三个电机，pitch2 保持白色
- `app/leg/leg_protocol.hpp`：控制帧删除自救位，功率 11 位 0.25 W/LSB 改为 12 位 0.125 W/LSB（4095 不限）；状态帧 3 字节改 4 字节，增加下板使能与 6 个 2 位电机状态
- `app/leg/leg_down/chassis.cpp`：FSi6X 在线时优先接管，否则按上板指令；不再透传自救；`chassis_write_can_frame()` 按下板电机对象填状态
- `app/leg/leg_up/chassis.cpp`：使能后首次 pause 上升沿置位锁存，之后 pause 切换腿长；失能或下板断联复位锁存
- `lib/robot/chassis_leg.*`：保留 `allow_rescue` 与 `set_allow_rescue()`；控制帧删除自救位，`control_by_up()` 固定授权，FSi6X 保留 SWB 授权；`is_enable` 移到 public 供状态帧读取
- `lib/robot/gimbal_classic.*`：删除 `use_angle_loop`，始终运行角度外环；`leg_up/gimbal.hpp` 同步删除配置项
- `agents/03-control/shooter/gimbal_imu_test.cpp`：Fixture 改为用角度 PID 的 kp 模拟关闭外环，删除速度模式下外环目标跟随实测的断言

## 协议

- 控制帧 8 字节：bit0 使能，bit1 跳跃，bit2..14 vx，bit15..30 wr，bit31..43 yaw，bit44..52 腿长，bit53..63 功率
- 状态帧 4 字节：bit0 倒地，bit1 下板使能，bit2..17 wr，bit18..29 依次为左前、左后、右后、右前、左轮、右轮的 2 位状态（0 正常、1 过热、2 达妙错误、3 离线），bit30..31 保留

## 行为变化

- 上板使能需要多按一次 pause；副驾 FSi6X 上电后下板会顶掉上板控制
- 上板控制时下板固定允许自救，不再随上板状态撤销；FSi6X 仍由 SWB 下挡决定是否允许
- 下板断联时 UI 的 w1~w4、s1~s4 显示粉色；下板在线未使能时显示白色

## 验证

- `python agents/04-leg/chassis/test_leg_up_control.py`：通过，覆盖使能锁存、腿长切换、断联复位、两帧位布局与编解码
- `build/leg-leg_up`、`build/leg-leg_down`、`build/jelly_sentry`、`build/steer_sentry`、`build/test` 全部链接通过
- clang-format 检查通过
- `python agents/03-control/shooter/verify_gimbal_imu.py` 在 `test_speed_feedback` 的 `chassis_wr` 断言失败，基线 `a698a27` 同样失败，属于旧断言与当前 GimbalClassic 不再扣底盘转速的差异，与本次改动无关

## 待上车核对

- pitch2 图元坐标、下板电机状态与实际故障的对应（尤其达妙 `err_code` 与过热阈值 80 ℃）
- 首次 pause 使能的操控手感；FSi6X 与上板同时在线时的优先级是否符合预期
- 功率 0.125 W/LSB 后下板接收的功率目标与超电实测一致
