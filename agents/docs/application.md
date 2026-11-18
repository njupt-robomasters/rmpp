# 应用组织约定

仅在修改 `app/**` 时读取本文档

## 职责边界

- app 选择硬件和车型策略
- lib 接收句柄或引用
- 板卡 BSP 实现外设
- 校准组合键、断联保持使能等车型策略在 app 展开
- yaw 校零与偏移持久化在 app 直接操作电机和 KvStore：infantry 使用 yaw，sentry 使用 yaw1；启动将 `yaw_offset` 读入 `motor.config.offset`，校准时按连续输出轴角度 angle_multi、方向与减速比更新 offset，使单圈与多圈反馈同时归零，随后保存；两处 yaw 减速比均为 1
- lib 不为车型策略保留开关或流程
- 发射机构、底盘、超级电容、遥控器、裁判通信与 MiniPC 通信按值保存只读 `Config`，直接读取配置字段；PID 状态、运动学矩阵、校准值与其他运行状态独立保存，超级电容 RLS 的固定参数读取 `config.rls`
- 云台、17 mm 发射机构及裁判通信的配置副本为 private；17 mm 的 motor 与 42 mm 的弹速、摩擦轮速度仅供类内及 SWD 使用。矩阵与舵轮底盘的 vx/vy 供派生类控制使用，权限为 protected；应用读取的 wr 保持 public
- `GimbalClassic` 与 `GimbalDualYaw` 独立实现，控制流程在各自 `update()` 中展开，按值保存只读 `Config`；Classic 固定采用电机轴 IMU 速度内环，通过 `use_angle_loop` 选择世界角度外环，DualYaw 保留 ECD/IMU 模式和编码器目标限位
- Classic 的 `pitch_min`、`pitch_max` 经 IMU 与编码器偏差换算后钳制世界 pitch 目标，并按编码器实测角度保护最终输出；DualYaw 用它们钳制编码器目标
- 云台和 17/42 mm 发射机构通过 `Motor::set_output()` 设置原生输出，PID 单位和限幅由 app 按电机配置；PID 自身的输出限幅与积分回算保留，Classic 的 pitch ECD 保护截断输出后额外调用 `track(0)` 回算积分
- 双 yaw 共用速度 PID 输出，各电机按自身原生单位接收同一数值；sentry 的 DM4310 为 N·m、GM6020 为 A，参数待实车整定
- 17 mm 拨弹在 `update()` 内使用 STOP/FEED/REVERSE 状态机，失能、热量保护、零弹频或拨弹离线优先停止；拨弹独立于摩擦轮开关和在线状态
- 卡弹检测与退弹分别用 `block_elapsed`、`reverse_elapsed` 累加限幅后的 `dt`；连续命令电流饱和达到 `block_time` 后反向运行 `reverse_time`，方向和速度跟随当前有符号弹频；STOP 清累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID
- 17 mm 拨弹先由状态机确定目标速度，再统一计算速度 PID；FEED 使用上一周期命令电流检测卡弹，状态切换在下一周期改变目标速度

## GimbalClassic 控制与观测

- 机构为底盘→yaw→pitch，IMU 固连 pitch 随动部分；`update(q, gyro)` 接收云台到世界的单位四元数和完成安装修正的云台系角速度（X 沿枪口、Y 沿 pitch 正轴）。pitch 编码器零位对应两级坐标系对齐
- 内环：`pitch_joint = AngleAxisf(motor.pitch.angle, UnitY())` 把 gyro 旋转到 yaw 随动系，Z/Y 分量即 yaw、pitch 电机轴的绝对角速度，作为 `speed_measure`；电机编码器速度只供观察
- 外环：`angle_ref/angle_measure` 为枪口世界 yaw/pitch（ZYX 欧拉角）。ZYX 的 yaw/pitch 完整描述枪口指向，roll 只是绕枪口自旋，两轴机构无法也无需控制，因此外环直接对欧拉角误差闭环（yaw 误差经 `Angle` 回绕），省去目标四元数与 roll 参考
- 外环加速度规划：`Config` 的 `yaw_max_accel`、`pitch_max_accel` 默认 `INFINITY`（原样传入误差，等效禁用规划）；设为有限正数时，在线性区阈值 `e0 = max_accel / kp²` 内保持原误差，超出后经 `plan_angle_error()` 折算为 `±√(e0·(2|e| − e0))` 再送入角度 PID，使比例项沿恒减速度平方根制动曲线 `±√(2·max_accel·(|e| − e0/2))` 平滑收敛，在 `e0` 处值与斜率连续，消除阶跃目标与外力大扰动下的制动饱和超调
- `set_speed()` 为电机轴速度，直接作为 `speed_ref` 的前馈；外环开启时同时积分到世界角度目标（底盘水平时两者一致，倾斜时的差异由外环修正）。外环输出的世界 yaw/pitch 角速率修正合成 `ψ̇·Z + θ̇·(−sinψ, cosψ, 0)` 后经 `pitch_joint * q⁻¹` 换算到电机轴再叠加，底盘倾斜时消除轴间耦合，roll 不引入稳态偏差。pitch 接近 ±90 度时世界 yaw 奇异，实际实例由 pitch 保护范围避开
- `use_angle_loop = false` 只保留速度内环；角度目标跟随实测，`set_angle()` 仅清零速度指令
- 使能与在线判断集中为 `can_pitch = is_enable && pitch 在线`、`can_yaw = can_pitch && yaw 在线`（yaw 换算依赖 pitch 关节角）。外环由私有成员 `update_angle_loop()` 直接读写两轴状态和角度 PID：不可控、关闭外环或首次 `update()` 时目标跟随实测并清空角度 PID，恢复后自然从当前姿态开始，无需上周期状态；速度内环不可控时清零目标、清空 PID、输出 0
- `angle_ref/angle_measure`、`speed_ref/speed_measure` 即两级闭环的最终目标与反馈：`speed_ref` = 电机轴速度指令 + 换算后的外环修正，yaw 再减 `chassis_wr` 抵消底盘带动；速度误差统一为 `speed_ref − speed_measure`
- pitch 目标限位：外环每周期取 `offset = pitch.angle_measure − motor.pitch.angle`，把编码器限位平移为世界 pitch 范围 `[pitch_min + offset, pitch_max + offset]`，钳制速度积分后的目标，`set_angle()` 的越界目标在下一次 `update()` 同样钳制；目标停在边界后不再累积，避免误差经 `Angle` 回绕后反转到另一侧。目标被钳住且 `pitch_speed` 向外时从 `speed_ref` 撤掉该前馈，否则平衡点会冲出限位约 `pitch_speed / kp`；向内指令保留前馈。底盘倾斜时世界 pitch 与关节角的二阶非线性由每周期刷新的 offset 吸收
- pitch 输出保护兜底超调：编码器 `<= pitch_min` 时禁止负输出、`>= pitch_max` 时禁止正输出，截断后 `track(0)` 回算积分，保留回程方向；只开速度环时仅有此保护
- 主机测试与 ARM 对象编译入口见 `agents/tasks/11-shooter_state/verify_gimbal_imu.py`；DualYaw 保持 ECD/IMU 模式

## 启动与循环

- CubeMX 初始化后依次执行 `bsp_init()`、`app_main()`
- `app.cpp` 分为 `setup()`、`loop()`、`app_main()`：`setup()` 放一次性初始化；`loop()` 依次轮询各设备与模块；`app_main()` 先调用 `setup()`，再用文件作用域 `static Timer app_timer` 按 1 kHz 调度 `loop()`，并用 `static Monitor app_monitor` 统计频率与占用（SWD 观察）
- 计时器、分频计数等跨周期状态放在文件作用域 `static`，函数内不定义 `static` 变量
- 低于 1 kHz 的任务在 `loop()` 内用 `<用途>_timer` 分频后提前返回，如 cdc 示例的 `print_timer`
- 自己调用 `Pid` 的 app 函数用 `update_timer` 计一次周期并限幅后传给各 PID
- dummy 和 leg 用 `uint32_t` 计数递增、取余分频，无手动清零：dummy 的 yaw 每 2 次循环发送，超电每 10 次循环发送，模拟裁判每 100 次循环结算；leg 左右关节交替发送，每个关节 500 Hz，底盘只在发送左腿的一拍计算（500 Hz），保证计算时两侧关节反馈都已刷新
- 使用普通 loop，不引入操作系统或整车包装类

## 文件组织

- 公共内容放在 `app.hpp`
- 各模块在同名 `.hpp` 声明、`.cpp` 实现
- 文件名不加 `_loop` 后缀
- 配置和实例跟随所属模块
- leg_down 的 LQR 系数独立放在 `app/leg_down/lqr_fit.hpp`，由 `agents/tasks/09-leg_lqr/gen_lqr_fit.py` 生成；`chassis.hpp` 只引用，当前参数与验证范围见任务 09
- 组件可独立使用
- 整合管理器引用外部实例

## 遥控器映射

- 不设置遥控器汇总层
- 底盘、云台、发射分别在对应 `.cpp` 直接读取 VT13、FSi6X
- 操控参数和跨周期状态使用文件内 `static`

## 持有与更新

- 设备由 app 定义；只服务一个机构的设备（电机）由该机构持有并在其 `update()` 中更新
- 被多处使用或与机构业务无关的设备（姿态 `Attitude`、超电 `SuperCap`）由 app 调用 `update()`，结果再交给机构：每周期都要刷新的测量经机构 `update(...)` 参数传入（如底盘和 Classic 云台 `update(q, gyro)`、DualYaw `update(yaw, pitch)`），其余输入用 `set_*`（如 `set_power_ref()`、`set_gimbal_yaw()`、`set_chassis_wr()`）
- app 先更新共享设备再更新机构：如 `super_cap.update()` → `chassis.set_power_ref(super_cap.power_chassis_ref)` → `chassis.update()`
- 发送（`send()`、`write_can_frame()` 组帧）始终由 app 安排
- 模块间不互相引用状态
- 多模块共用判断放在 `app.hpp`
