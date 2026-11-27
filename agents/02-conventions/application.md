# 应用组织约定

仅在修改 `app/**` 时读取本文档

## 职责边界

- app 选择硬件和车型策略
- lib 接收句柄或引用
- 板卡 BSP 实现外设
- 校准组合键、断联保持使能等车型策略在 app 展开
- yaw 校零与偏移持久化在 app 直接操作电机和 KvStore：infantry 使用 yaw，steer_sentry、jelly_sentry 使用 yaw1；启动将 `yaw_offset` 读入 `motor.config.offset`，校准时按连续输出轴角度 angle_multi、方向与减速比更新 offset，使单圈与多圈反馈同时归零，随后保存；两处 yaw 减速比均为 1
- lib 不为车型策略保留开关或流程
- 发射机构、底盘、超级电容、FSi6X、裁判通信与 MiniPC 通信按值保存只读 `Config`，直接读取配置字段；PID 状态、运动学矩阵、校准值与其他运行状态独立保存，超级电容 RLS 的固定参数读取 `config.rls`
- 云台、17 mm 发射机构及裁判通信的配置副本为 private；17 mm 的 motor 与 42 mm 的弹速、摩擦轮速度仅供类内及 SWD 使用。矩阵与舵轮底盘的 vx/vy 供派生类控制使用，权限为 protected；应用读取的 wr 保持 public
- `GimbalClassic` 与 `GimbalDualYaw` 独立实现，控制流程在各自 `update()` 中展开，按值保存只读 `Config`；Classic 固定采用电机轴 IMU 速度内环并叠加世界角度外环，DualYaw 保留 ECD/IMU 模式和编码器目标限位
- Classic 的 `pitch_min`、`pitch_max` 经 IMU 与编码器偏差换算后钳制世界 pitch 目标，并按编码器实测角度保护最终输出；DualYaw 用它们钳制编码器目标
- 云台和 17/42 mm 发射机构通过 `Motor::set_output()` 设置原生输出，PID 单位和限幅由 app 按电机配置；PID 自身的输出限幅与积分回算保留，Classic 的 pitch ECD 保护截断输出后额外调用 `track(0)` 回算积分
- GimbalDualYaw 的双 yaw 共用速度 PID 输出，各电机按自身原生单位接收同一数值；steer_sentry 的 DM4310 为 N·m、GM6020 为 A，参数待实车整定
- 17 mm 拨弹在 `update()` 内使用 STOP/FEED/REVERSE 状态机，失能、热量保护、零弹频或拨弹离线优先停止；拨弹独立于摩擦轮开关和在线状态
- 卡弹检测与退弹分别用 `block_elapsed`、`reverse_elapsed` 累加限幅后的 `dt`；上一周期拨弹 PID 输出绝对值达到 `shoot_pid_config->max_out`（饱和）并持续 `block_time` 后反向运行 `reverse_time`，阈值与输出同单位（随电机 `set_output()`：DJI 为 A、EL05 为 N·m）；纯 P 参数下堵转输出为 kp×目标转速，低于 max_out 时检测不到，需靠积分推到饱和。方向和速度跟随当前有符号弹频；STOP 清累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID
- 17 mm 拨弹先由状态机确定目标速度，再统一计算速度 PID；FEED 使用上一周期 `shoot_output` 检测 PID 输出饱和，状态切换在下一周期改变目标速度

## 底盘离线反馈

- Motor 超时后 angle 清零，angle_multi 保留；轮腿接受离线关节角清零后的解算变化，双 yaw 的离线控制逻辑后续单独调整
- 舵轮仅在轮、舵均在线时允许对应轮输出；仅轮离线时，舵仍跟随速度逆解得到的目标角，零速度时保持目标角
- 速度正解排除离线位置：全向轮／麦轮按有效运动学行最小二乘求解，通常至少 3 轮；舵轮在无侧滑假设下按有效位置的 x/y 速度约束求解，至少 2 个不同位置。正常四轮舵轮解算与原均值算法等价，缺轮时重新求解平移与旋转，避免偏置
- 约束不足或秩亏时 vx/vy/vz/wr.measure 为 NaN；力控清空底盘速度 PID 并关闭轮输出，舵向控制继续；轮速控制仍按在线轮执行。恢复足够反馈后自动恢复解算
- 总牵引力反馈需要全部轮数据，缺失时 fx/fy/fz.measure 为 NaN；输出力求和与 PID 回算保持独立。steer_sentry 对未知 wr 关闭云台转速前馈；jelly_sentry 的 yaw 闭环直接使用 IMU 反馈，现有 UI 数字字段显示 0，底盘测量本身保留 NaN
- leg_up 跟随模式下发计入车头偏角的航向误差；小陀螺或 yaw 电机离线时发 0

## GimbalClassic 控制与观测

- 机构为底盘→yaw→pitch，IMU 固连 pitch 随动部分；`update(q, gyro)` 接收云台到世界的单位四元数和完成安装修正的云台系角速度（X 沿枪口、Y 沿 pitch 正轴）。pitch 编码器零位对应两级坐标系对齐
- 内环：`pitch_joint = AngleAxisf(motor.pitch.angle, UnitY())` 把 gyro 旋转到 yaw 随动系，Z/Y 分量为 yaw、pitch 电机轴的绝对角速度，作为各自的 `speed_measure`；电机编码器速度只供观察
- 外环：`angle_ref/angle_measure` 为枪口世界 yaw/pitch（ZYX 欧拉角）。ZYX 的 yaw/pitch 完整描述枪口指向，roll 只是绕枪口自旋，两轴机构无法也无需控制，因此外环直接对欧拉角误差闭环（yaw 误差经 `Angle` 回绕），省去目标四元数与 roll 参考
- 外环加速度规划：`Config` 的 `yaw_max_accel`、`pitch_max_accel` 默认 `INFINITY`（原样传入误差，等效禁用规划）；设为有限正数时，在线性区阈值 `e0 = max_accel / kp²` 内保持原误差，超出后经 `plan_angle_error()` 折算为 `±√(e0·(2|e| − e0))` 再送入角度 PID，使比例项沿恒减速度平方根制动曲线 `±√(2·max_accel·(|e| − e0/2))` 平滑收敛，在 `e0` 处值与斜率连续，消除阶跃目标与外力大扰动下的制动饱和超调
- `set_speed()` 为电机轴速度，直接作为 `speed_ref` 的前馈；外环开启时同时积分到世界角度目标（底盘水平时两者一致，倾斜时的差异由外环修正）。外环输出的世界 yaw/pitch 角速率修正合成 `ψ̇·Z + θ̇·(−sinψ, cosψ, 0)` 后经 `pitch_joint * q⁻¹` 换算到电机轴再叠加，底盘倾斜时消除轴间耦合，roll 不引入稳态偏差。pitch 接近 ±90 度时世界 yaw 奇异，实际实例由 pitch 保护范围避开
- 不需要关闭角度外环的开关：角度 PID 的 `kp` 设为 0 时外环修正为零，等效只有速度内环，角度目标仍随速度指令积分
- 使能与在线判断集中为 `can_pitch = is_enable && pitch 在线`、`can_yaw = can_pitch && yaw 在线`（yaw 换算依赖 pitch 关节角）。外环由私有成员 `update_angle_loop()` 直接读写两轴状态和角度 PID：不可控、关闭外环或首次 `update()` 时目标跟随实测并清空角度 PID，恢复后自然从当前姿态开始，无需上周期状态；速度内环不可控时清零目标、清空 PID、输出 0
- `angle_ref/angle_measure`、`speed_ref/speed_measure` 即两级闭环的最终目标与反馈：`speed_ref` = 电机轴绝对速度指令 + 换算后的外环修正，速度误差为 `speed_ref − speed_measure`，保持绝对速度指令与世界角度闭环；底盘旋转由 yaw 速度环自然抵消，不再需要底盘转速前馈
- pitch 目标限位：外环每周期取 `offset = pitch.angle_measure − motor.pitch.angle`，把编码器限位平移为世界 pitch 范围 `[pitch_min + offset, pitch_max + offset]`，钳制速度积分后的目标，`set_angle()` 的越界目标在下一次 `update()` 同样钳制；目标停在边界后不再累积，避免误差经 `Angle` 回绕后反转到另一侧。目标被钳住且 `pitch_speed` 向外时从 `speed_ref` 撤掉该前馈，否则平衡点会冲出限位约 `pitch_speed / kp`；向内指令保留前馈。底盘倾斜时世界 pitch 与关节角的二阶非线性由每周期刷新的 offset 吸收
- pitch 输出保护兜底超调：编码器 `<= pitch_min` 时禁止负输出、`>= pitch_max` 时禁止正输出，截断后 `track(0)` 回算积分，保留回程方向；只开速度环时仅有此保护
- 主机测试与 ARM 对象编译入口见 `agents/03-control/shooter/verify_gimbal_imu.py`；DualYaw 保持 ECD/IMU 模式

## 启动与循环

- CubeMX 初始化后依次执行 `bsp_init()`、`app_main()`
- `app_main()` 先调用 `setup()` 完成一次性初始化，再用循环内的局部 `static Timer app_timer` 按 1 kHz 调度 `loop()`，并用局部 `static Monitor app_monitor` 统计整个周期的频率与占用（SWD 观察）。`loop()` 依次更新共享设备（RGB、蜂鸣器、遥控器、姿态、裁判）与各模块，最后调用 `send()`。`send()` 集中安排全部 CAN 发送与 `referee_sender.send()`，模块 `*_loop()` 内不发送；无发送任务的示例省略 `send()`
- 计时器、分频计数等跨周期状态通常放在文件作用域 `static`；主循环的 `app_timer` 放在 `while (true)` 内“1 kHz 更新”注释后，`app_monitor` 放在“负载监控”注释后，均保留 `static`
- 低于 1 kHz 的任务在 `loop()` 内用 `<用途>_timer` 分频后提前返回，如 cdc 示例的 `print_timer`
- 自己调用 `Pid` 的 app 函数用 `update_timer` 计一次周期并限幅后传给各 PID
- 发送分频用 `uint32_t send_cnt` 递增取余，无手动清零，通常为 `send()` 内 `static`：dummy 的 yaw 每 2 拍、超电每 10 拍发送，模拟裁判每 100 次循环结算；leg_down 左右关节交替发送，每个关节 500 Hz，底盘只在发送左腿的一拍调用 `chassis_loop()`（500 Hz），保证计算时两侧关节反馈都已刷新，`attitude.update()` 仍在 `loop()` 每周期执行；leg_down 的 `loop()` 需读取同一拍数，`send_cnt` 为文件作用域 `static`，在 `send()` 末尾递增。jelly_sentry 运行时可写周期的电机另用 `send()` 内 `static` 分频计数，周期 0、1 均为每拍发送
- C 板 CAN 只有 3 个发送邮箱、无软件队列，满时丢弃；每路每拍至多 3 帧。steer_sentry 的 CAN1 每拍发大 yaw 与 0x1FF（小 yaw、拨弹、舵 1/2），第 3 帧由 0x200（轮）、0x2FF（舵 3/4）、裁判发送按 `send_cnt % 3` 轮流，各约 333 Hz；底盘电机回传 125 Hz。leg_up 的 CAN1 奇拍发底盘控制帧（500 Hz），偶拍中每 10 拍发超电（100 Hz），其余偶拍交给裁判发送（至多 400 Hz）；mc02 的 FDCAN 带发送 FIFO，jelly_sentry 不受此限
- 使用普通 loop，不引入操作系统或整车包装类

## 文件组织

- 公共内容放在 `app.hpp`：共享设备（蜂鸣器、RGB、KvStore、遥控器、上位机）与多模块共用判断；各模块 `.cpp` 先包含自身头文件，再按需包含 `app.hpp` 与其他模块头文件；不设 `misc` 文件，RGB 在 `app.cpp` 的 `loop()` 内更新
- 裁判系统与 UI 放在 app 层 `referee.hpp`、`referee.cpp`：头文件定义 `referee`、`referee_sender`、`ui` 实例并声明 `referee_loop()`；`referee_loop()` 依次 `referee.update()`、`set_robot_id()`，F 键按下沿 `ui.init()`，锁存受击方向并填写 `ui.update()`；`referee_sender.send()` 在 `send()` 中调用。dummy 的 `referee.*` 为模拟裁判功率结算
- 各模块在同名 `.hpp` 声明、`.cpp` 实现
- 文件名不加 `_loop` 后缀
- 配置和实例跟随所属模块
- leg_down 的 LQR 系数放在 `lib/robot/chassis_leg_lqr.hpp`，由 `agents/04-leg/lqr/gen_lqr_fit.py` 生成；`app/leg/leg_down/chassis.hpp` 只引用，当前参数与验证范围见 `agents/04-leg/lqr/`
- 组件可独立使用
- 整合管理器引用外部实例

## 遥控器映射

- VT13 在 `update()` 内把遥控器与键鼠合成为操控量，app 优先读取：`is_enable`（N、S 挡）、`vx`/`vy`/`wr`、`yaw_speed`/`pitch_speed`、`bullet_speed`、`bullet_freq`、`is_aim`；原始量在 `rc`、`mouse`、`key`，供 fn、pause、校准组合键等 app 专用操作读取
- 平移与旋转为摇杆或拨轮乘满量程，叠加 W/S、A/D、Shift/Ctrl 键盘斜坡后限幅；按反向键时同向速度先清零再反向加速，松手按减速度回零；加减速度设 `INFINITY` 时一个周期到满量程或归零。斜坡用本类 dt，`DT_MIN`/`DT_MAX` 限幅
- 云台为摇杆叠加鼠标，无斜坡，限幅到满量程；pitch 抬头为负，与 Classic 云台一致。按住 trigger 时 pitch 摇杆改为弹频倍率（负值反转），否则鼠标左键按弹频；S 挡输出配置弹速；photo 或鼠标右键为自瞄
- 弹频由鼠标滚轮调节：`mouse_z` 正值为向前/向上滚、负值为向后/向下滚（通信协议 2.2.1）；`last_wheel` 记录上周期值，仅当上周期为 0 且本周期的符号为负/正时减/加一步，连续同号或未经 0 直接换号都不重复触发，结果限幅到 [`bullet_freq_min`, `bullet_freq_max`]
- 断联时原始量、键盘斜坡与操控量全部清零，挡位为 ERR
- 图传链路的裁判帧与遥控帧混在 VT13 串口：VT13 同一缓冲区按帧头分别校验，0xA5 帧只处理 0x0311（载荷至多 60 B），在串口接收中断内依序调用 `add_0311_callback()` 注册的回调，回调在启动前注册；0x0311 不影响 `is_connect`。机器人发往自定义终端的 0x0310 同样走图传串口，目前未提供发送接口；Referee 不再收发图传链路报文
- VT13 回调按“协议接收”统一写法（`conventions.md`）在 `rx_fifo` 中分流：`A9 53` 遥控帧按固定长度验 `crc16_referee`、更新 `raw` 和在线计时；`A5` 裁判帧先验 5 字节帧头的 `crc8_referee`，再按载荷长度验整帧 `crc16_referee`。半帧原样保留，错位、超长或校验失败时丢弃 1 字节
- VT13 的 `Config` 公开且可修改，下次 `update()` 生效；滚轮在 `update()` 内按 `bullet_freq_step` 增减 `config.bullet_freq`；各 app 在实例处给出满量程、加减速度、弹速与弹频默认值
- steer_sentry（VT13 接 USART6）与 jelly_sentry（VT13 接 USART1）只用 VT13，已移除 FSi6X；控制方式见下节“哨兵控制”。jelly_sentry 云台 pitch 取 `-vt13.pitch_speed`，保持迁移前摇杆上推目标增大的方向；弹频由鼠标滚轮调节，调节后的值同时用于手动和自主射击
- FSi6X 只保留在 dummy、leg_down 断联回退与 `examples/rc`，在对应 `.cpp` 直接读取

## 哨兵控制

- jelly_sentry 的 app 接口统一为 `rc_fn()`、`sentry_active()` 等上游命名，自主行为集中在 `sentry.cpp`。正赛或 fn 锁存测试进入自主行驶；赛场自检与倒计时底盘零速待命、云台扫描，自检摩擦轮停转，5 s 倒计时预开摩擦轮，两个阶段拨弹均为 0
- jelly_sentry 在线 VT13 的 C 挡在所有阶段优先使底盘、云台和发射机构失能；否则按遥控和赛程使能。发射机构采用上游正常遥控逻辑：N 挡保持使能但摩擦轮弹速为 0，S 挡开摩擦轮，trigger 或左键给弹频，人工自瞄需视觉开火许可；删除双击强制开火
- 两车导航速度均按大 yaw 雷达系接收，经负小 yaw 角转到云台系。jelly_sentry 导航在线时始终叠加 VT13 平移，正赛同样叠加，不额外限幅；steer_sentry 叠加后按每轴 ±5 m/s 限幅。两车任一 yaw 反馈丢失时自主底盘停车
- jelly_sentry 保留 `NavState`（`IDLE / GO_CENTER / AT_CENTER / GO_HOME`）：正赛或 fn 测试默认去中心 `(6.0, -3.8)` m，裁判 `0x0209` bit23 判到达中心；有效血量 `<100` 回家 `(0.15, -0.15)` m，回满 `maximum_HP` 再出击。点位状态和目标写入集中在 `sentry.cpp`，`minipc_loop()` 更新协议与最近有效过洞状态
- jelly_sentry 非过洞时按合成平移速度 0～5 m/s 插值 90～60 rpm 小陀螺，只限幅旋转插值输入。过洞时随头、平移照常、暂停扫描。普通导航断联后平移归零、原地 90 rpm，云台继续扫描或自瞄；过洞断联后平移与旋转均归零，保持云台目标并暂停人工转动、自瞄跟踪和扫描，等待新有效导航帧。应用保存最近有效过洞标志，Nav 本身按协议清零
- steer_sentry 自主叠加遥控平移及旋转，非过洞为 120～60 rpm 小陀螺，导航超时后导航量清零、遥控仍可用。血量 ≤120 回家，≥400 再去中心，导航坐标沿用其应用配置
- 云台人工控制采用摇杆叠加鼠标，R 键掉头，photo 或右键且识别到目标时跟踪。jelly_sentry 自主优先处理过洞断联保持，场下有人工速度输入时退出锁定并接管；仅 `GO_CENTER / AT_CENTER` 允许 5 m 内自瞄锁定与开火，回家及自检/倒计时取消锁定并停拨弹，扫描为 yaw 180°/s、pitch 60°/s（编码器 −0.1～0.314 rad 往返）
- 小 yaw 保留 ±50° 自瞄优先窗口：允许交战且当前识别到目标时启用；进入窗口保存大轴世界朝向，目标在窗口内时大轴保持，越界由大轴补足。退出窗口后大轴持续追头部目标，小轴回中；过洞断联时冻结原分配模式
- jelly_sentry 默认弹速 18 m/s、弹频 10 Hz，遥控 yaw/pitch 满量程 720°/s；滚轮在 5～10 Hz 间调弹频。每周期用 `shooter.set_heat(剩余热量, 冷却速率)` 设置裁判输入，再 `update()` 计算热量允许的弹频；默认延迟 0.3 s、热量余量 10、降频增益 0.25
- yaw 校零在 C 挡按住 fn 与 photo 的按下沿触发，按大 yaw `angle_multi` 更新 offset 并写入 KvStore；steer_sentry 沿用应用标记 `sentry`

## 持有与更新

- 设备由 app 定义；只服务一个机构的设备（电机）由该机构持有并在其 `update()` 中更新
- 被多处使用或与机构业务无关的设备（姿态 `Attitude`、超电 `SuperCap`）由 app 调用 `update()`，结果再交给机构：每周期都要刷新的测量经机构 `update(...)` 参数传入（如底盘和 Classic 云台 `update(q, gyro)`、DualYaw `update(yaw, pitch)`），其余输入用 `set_*`（如 `set_power_ref()`、`set_gimbal_yaw()`、`set_leg_length()`）
- app 先更新共享设备再更新机构：如 `super_cap.update()` → `chassis.set_power_ref(super_cap.power_chassis_ref)` → `chassis.update()`
- 发送（`send()`、`write_can_frame()` 组帧）始终由 app 安排
- 模块间不互相引用状态
- 多模块共用判断放在 `app.hpp`

## jelly_sentry 超级电容

- 已装超电，CAN1，沿用 `SuperCap` 默认命令 ID 0x061、反馈 ID 0x051；整车输入使用真实裁判功率上限与缓冲能量
- `chassis_loop()` 采用上游条件 `vt13.is_enable || sentry_in_game()` 同时设置 DCDC 和放电补偿使能，随后更新裁判输入与超电，再把 `power_chassis_ref` 交给四轮力控底盘；发送由 `app.cpp::send()` 每 10 拍执行一次，目标 100 Hz
- 超电控制与整车执行器使能分别计算，不再以裁判在线和整车使能额外门控，也不单列赛前只充电分支；`SuperCap` 内部继续处理反馈超时、缓冲能量与放电电压约束
- 超电实测功率覆盖范围按实车供电支路核对，四轮模型只估计轮电机功率；UI 的 CAP 弧长和整数百分比取 `SuperCap::energy_ratio()`，按（当前电容电压 / 满电电压）² 计算，满电默认 23.85 V，离线显示 0
- CAP、RPM、BULLET、SHOOT 四段仪表圆弧始终为绿色；整车图中的超电状态框按超电状态着色
- 实施与验证记录见 `agents/07-power/super_cap/README.md`，硬件联调由用户审计后决定

## jelly_sentry 双传送带驱动

- 发射机构服从整车使能，C 挡在所有阶段优先失能。正常遥控采用上游逻辑：N 挡保持使能、摩擦轮弹速为 0；S 挡运行两路电机（`vt13.bullet_speed`），拨盘取 `vt13.bullet_freq`，人工自瞄需视觉开火许可。自主时摩擦轮常开；回家与赛前禁发弹，自检停摩擦轮、5 s 倒计时预开摩擦轮
- 用户说明 rub1/rub2 为工业传送带转动机构，工程沿用摩擦轮命名。两个 P630 由 app 定义为 `Vesc`，绑定 CAN2，节点 ID 为 `0x21/0x22`
- `Shooter17mm::update()` 通过 `set_speed()` 设置输出轴目标角速度（rad/s），由 Vesc 换算为 ERPM，使用电调内部速度环
- 整车每 10 ms 发送两路 VESC 指令（各 100 Hz），使能时速度扩展帧 ID 为 0x321/0x322、4 字节大端 ERPM，失能时发送零电流帧
- Vesc 最大电流为 20 A、reduction 为 7（包含极对数）；rub1 方向 true、rub2 方向 false，实际极对数待核对

## jelly_sentry EL05

- 拨弹 EL05 绑定 CAN2，按 old 分支移植私有扩展帧协议，电机节点0x12、主机节点0x22；使能ID 0x03002212、停止ID 0x04002212、无故障运行反馈ID 0x02801222。两路pitch仍用原标准帧协议
- 电机端配套参数：CAN_ID=18、CAN_MASTER=34、protocol_1=0、CAN_TIMEOUT=20000（约1秒）；此前上位机写入记录为protocol_1=2，需用户核对并改为0后断电重启，主控不在线改参
- 整车shoot实例控制计算保持1ms，EL05每2拍发送（2ms、500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答
- 按用户实测反向要求，shoot在shooter.hpp中显式配置is_invert=true，同步反向力矩指令和位置/速度/力矩反馈；三路方向可分别观察rub1.config.is_invert、rub2.config.is_invert和shoot.config.is_invert
- 卡滞参数采用上游默认 block_time=0.25 s、reverse_time=0.1 s，拨弹 PID 输出上限为6 N·m；9发/转，实际堵转与退弹效果待上车核对

## jelly_sentry 上游接口适配

- `GimbalFold` 位于 `lib/robot/gimbal_fold.*`，目前仅 jelly_sentry 使用；app 每周期更新 attitude，通过 `gimbal.update(attitude.q, imu.gyro)` 传入四元数与安装修正后的机体系角速度，yaw1 零点由 app 经 KvStore 持久化
- yaw1/yaw2 反馈按上游：小 yaw 以头部世界 yaw 为外环反馈、`(q * gyro).z()` 经一阶低通后为速度内环反馈；大 yaw 以头部世界 yaw 减小 yaw 编码器角为外环反馈，以滤波后的世界 Z 角速度减小 yaw 编码器速度为内环反馈。`gimbal.hpp` 的 `yaw_gyro_fc` 当前为 50 Hz，0 直通；失能、离线与恢复首周期同步原始测量。删除 ECD 模式与 `set_chassis_wr()` 前馈，保留本地自瞄窗口
- yaw 轴要求大小 yaw 同时在线；不可控或恢复首帧目标跟随实测、清 PID、输出零。小 yaw 实测达到 ±50° 时截断向外输出，保留回程；窗口内大轴保持世界朝向，越界补足，退出窗口追头部目标使小轴回中。大轴速度分解按大小 yaw 轴近似世界竖直处理，倾斜/横滚工况需验证
- pitch2 保留本地控制：世界 pitch 目标按 IMU 与编码器差值换算，钳制编码器目标，位置环输出叠加未触限时的 `pitch_speed`，速度内环反馈为 `motor.pitch2.speed`；叠加重力补偿并跟踪电机实际限幅结果。保留速度 PID `kp=0.50558, ki=2, kd=0.02, ff=-1.55, max_i=0.4, max_out=5, err_fc=10` 与位置环 `kp=20, max_out=720°/s`
- 公开观测与配置接口统一上游命名：头部 `yaw.ref/measure/ecd_measure`、`pitch.ref/measure/ecd_ref/ecd_measure`。大小 yaw 当前四组 PID 参数已从 Ember 保存，数值及证据见 `agents/06-sentry/yaw_pid/README.md`；pitch2 参数保持原值。yaw 可选加速度规划默认 `INFINITY` 关闭
- IMU 按上游标定：传感器系零偏 XYZ 为 `(0.00412, 0.00166, 0.00212)` rad/s，重力模长 `9.80907 m/s²`，安装 yaw/pitch/roll 为 −90°/0°/180°，使用默认恒温 45 ℃
- 大 yaw、pitch1、pitch2 的原生输出单位为 N·m，小 yaw 7025 为 A；pitch1 固定失能。底盘由 app 更新超电后传入 `power_chassis_ref`
- 受击方向与实测弹速读取裁判公开只读的 `hurt_data`、`hurt_sequence`、`shoot_data`；受击按装甲 ID 换算到 IMU 参考系锁存 1 s，再转到头部方向显示
- 两台哨兵与 leg_up 在 `referee_loop()` 填写 `ui` 公开字段，再 `ui.update(referee_sender)`。电机状态：离线粉、主动失能白、过热橙、达妙错误黄、正常绿；未使用槽位白。`aim` 显示未使能白、在线绿、断联粉。超电离线粉、OFF 白、CHARGE 青、BOOST 绿，低于放电截止值时青色；弹频提示条满量程 10 Hz
- 独立 `examples-motor` 台架示例通过 SWD 查看 `motor_status_color`：断连粉、失能或零目标白、过热橙；其余状态下转速偏差超过 5 rpm 为青，达标为绿。该观察量不参与实车 HUD

## jelly_sentry 导航与 pitch1

- CDC1 用于 Aim 收发，CDC2 接 Nav 的导航下行帧（`A3`，24 字节）。`minipc_loop()` 在 `nav.update()` 清零前保存最近有效过洞标志，点位和控制流程集中于 `sentry.cpp`
- 导航使用雷达系速度，负小 yaw 转到头部系后始终叠加遥控平移；普通导航为 90～60 rpm 小陀螺，过洞随头并停扫。普通断联原地 90 rpm、零平移，过洞断联停车并保持云台目标；任一 yaw 反馈离线时自主底盘停车
- pitch1 全程失能，清 PID 并写零输出，保留反馈与失能帧发送
- pitch2 编码器软限位为 −0.14～0.314 rad，扫描与目标限幅共用此范围；速度内环与 PID 保留本地。yaw1/yaw2 反馈与接口采用上游，保留本地 ±50° 自瞄窗口
- 协议、坐标公式、过洞标志和离线处理详见 `agents/05-comm/minipc.md`；验证见对应任务记录

## jelly_sentry CAN 发送调度

- CAN1 为大 yaw DM4310、底盘与超电；大 yaw 命令 ID 0x11、反馈 ID 0x10。CAN2 为 pitch1、pitch2、EL05 与两路 P630；CAN3 为小 yaw 7025，控制 ID 0x141、反馈 ID 0x241
- 大 yaw `is_invert=true`，小 yaw `is_invert=false`；小 yaw 中位原始角度 `offset=3.192 rad`，更新固件后低输出核对合成角与 IMU 航向方向和中位
- 控制计算 1 kHz；固定发送分频：大 yaw、pitch2 各 1 kHz，pitch1 250 Hz，EL05 500 Hz，小 yaw `send()` 500 Hz，双传送带与超电各 100 Hz
- 小 yaw 发送内部交替电流命令 0xA1 与读角度 0x94，各 250 Hz；pitch1 固定发送失能帧。失能、使能和清错帧替换当次发送，不额外增加频率
- Aim/Nav 分别经 CDC1/CDC2 错拍各 250 Hz；导航上行沿上游固定发送节拍，yaw1 离线时发送保持的最后角度
- CAN2 自动重发与发送 FIFO 沿用用户生成配置，本轮不修改 `.ioc` 或生成代码。上述频率为按 1 ms 主循环计算的目标值，真实发送、反馈周期与总线负载待上板核对

## 轮腿上下板通信

- 上板 MiniPC 用 CDC1 收发自瞄帧，每 5 拍上报云台姿态、阵营和裁判实测弹速。按住 photo 或鼠标右键且识别到目标时，云台跟踪视觉世界 yaw/pitch；目标丢失、断联或退出自瞄时恢复摇杆与鼠标控制；R 键按下沿 `turn_yaw(180°)`。自瞄模式下仅有目标且视觉批准时才放行 VT13 弹频。协议见 `minipc.md`

- 上下板 Can(1) 经滑环连接，与下板全部电机、上板管理的超电共用 CAN1；两板通过 `../leg_protocol.hpp` 引用共享文件 `app/leg/leg_protocol.hpp`，包含按发送方命名的 `UP_ID = 0x02`、`DOWN_ID = 0x03` 和报文布局。上板控制帧 8 字节（使能、批准自救、跳跃、速度、yaw、腿长、功率目标），500 Hz；下板状态帧 4 字节（底盘倒地、实测航向角速度、6 个电机状态），250 Hz
- 共享协议文件统一定义 `LegUpFrame`、`LegDownFrame`、物理单位、`pack()` 与 `unpack()`；`Raw` 位域与解码后成员同序，编解码在 `pack()`、`unpack()` 内用局部 `Raw` 完成，不保存原始帧；位域按 arm-none-eabi-gcc 布局，主机 MinGW 测试加 `-mno-ms-bitfields`。解包先检查 ID 和长度，失败保持原数据。腿长为连续数值 `leg_length`（m），以 9 位无符号毫米数传输，范围 0..0.511 m；协议中没有腿长档位表。功率目标 `power_ref` 11 位 0.25 W/LSB，四舍五入，有限值范围 0..511.5 W，2047 表示不限功率（`INFINITY`）。`LegProtocol<TxFrame, RxFrame>` 封装发送帧、接收帧、在线状态与超时：构造时向 Can 注册回调，`update()` 超时清接收数据，`send()` 组帧发送；上板用 `LegUpProtocol`、下板用 `LegDownProtocol`，两板 `chassis_loop()` 直接读写 `leg_protocol` 字段，发送仍按 `app.cpp` 的拍数调用 `leg_protocol.send()`；回归测试检查共享文件引用与编解码
- 上板在 VT13 使能且下板在线时，首次按下 Pause 使能底盘，之后按 Pause 按 0.14 → 0.22 → 0.33 → 0.22 → 0.14 m 往返切换腿长。Z/X/C 各自在按下沿直选低/中/高档；选低档后 Pause 向上切，选高档后向下切，选中档保留原方向。E 或 Fn 任一按住时下发跳跃位，由下板检测上升沿。失能或下板断联时清使能锁存、复位低档，恢复后重新等待 Pause；每周期记录 Pause、V 和三个腿长键的前值
- 自救批准：下板回传 `is_fallen` 期间，Pause 或 V 的按下沿置批准位，该次按下不切腿长；V 与 Pause 等价，都是批准一次。`is_fallen` 清除（站起完成、失能或断联）后撤销批准，再次倒地需重新批准。未使能或未倒地时按键不置位
- `vt13.vx/vy` 是云台坐标系下前/左速度，已合成摇杆与 W/S、A/D。跟随时，`|vy| > |vx|` 首次从正向进入侧身，按 `vy` 正/负锁存车头相对云台左/右 90°；横向反打只反向行驶。`|vx| > |vy|` 回正，等幅和松手保持。失能或下板断联清锁存，小陀螺期间保留偏角。偏角只用于底盘，云台世界角目标沿用原控制。
- 上板按实测云台相对底盘角 `a = yaw.angle` 投影车头速度 `vx*cos(a) - vy*sin(a)`；跟随时发归一化的 `a + 跟随偏角` 作航向误差，`wr` 发云台 `yaw_speed` 前馈（视觉跟踪时为 0）。小陀螺发手动 `vt13.wr` 并清零航向误差，yaw 电机离线也清零航向误差。
- 下板执行策略：FSi6X 在线时优先接管；否则直接使用上板已投影的前向速度、航向误差、角速度、跳跃与腿长，上板断联 0.1 s（`LEG_TIMEOUT`）后失能。FSi6X 路径保留拨杆操作，航向误差设为 0，保留 LQR 速度与位移反馈
- `is_enable` 由下板直接透传给 `ChassisLeg`，下板状态帧不再回传使能；FSi6X 在线时完全接管、忽略上板一切指令（含批准自救），自救授权仍由 SWB 下挡决定。上板控制时 `control_by_up()` 把批准位置位到 `allow_rescue` 锁存，站起完成或失能后清除，上板断联期间保持已批准状态；底盘使能、倒地且已授权空闲时启动自救，站起完成、失能或跳跃时清除自救状态
- 超电接上板 Can(1)，每周期读取裁判功率上限和缓冲能量并更新，指令按 100 Hz 发送。模式跟随 VT13 整车使能：失能 OFF，使能 BOOST。`super_cap.power_chassis_ref` 随控制帧下发，下板上板控制时经 `set_power_ref()` 交给 `ChassisLeg`，FSi6X 控制时设 `INFINITY` 不限功率
- 上板控制帧：bit0 使能，bit1 批准自救，bit2 跳跃，bit3..15 前向速度，bit16..30 航向角速度目标，bit31..43 yaw，前三个运动量分别按 0.001 m/s、0.001 rad/s、0.001 rad 编码；bit44..52 腿长，bit53..63 功率目标（0.25 W/LSB，2047 不限）。下板状态帧：bit0 倒地，bit1..15 实测航向角速度（int16，0.001 rad/s，±16.383 rad/s），bit16..27 为左前、左后、右后、右前、左轮、右轮的 2 位状态（0 正常、1 过热、2 达妙错误、3 离线），bit28..31 保留。
- 下板 Can(1) 每拍 3 帧，匹配 3 个发送邮箱：偶拍左腿两关节与双轮 0x200，奇拍右腿两关节加第 3 帧，第 3 帧由 0x003 状态帧与 `referee_relay.send()` 轮流使用，各 250 Hz
- 裁判系统接下板 USART1（115200），`app.hpp` 的 `RefereeRelay referee_relay` 原样转发字节流：CAN ID 与 leg_protocol 一样按发送板命名，0x00 为下板发送、0x01 为上板发送，类内按本板视角写作 `tx_id`、`rx_id`（与 Referee 对应）。串口收到的字节切成至多 8 字节的 0x00 帧，每次 `send()` 发 1 帧，容量 2 kB/s，约为 2027 协议常规链路定频报文的 2 倍；收到的 0x01 帧载荷拼接后在串口空闲时一次发出。缓冲区 `to_can` 1024 B（1 Hz 定频报文与触发报文叠加的最坏突发约 500 B）、`to_uart` 256 B（两次 `send()` 间至多积压 32 B），满时弹出最旧字节，丢失的字节数分别累计到 `to_can_drop`、`to_uart_drop`。上板用 `Referee({.can = Can(1)})` 接收（默认 `rx_id = 0`）、`RefereeSender({.can = Can(1)})` 发送（默认 `tx_id = 1`）；发送队列保留最新 20 个待发请求，已组帧的数据继续发完。leg_up 的 `referee_loop()` 在 `referee.update()` 后调用 `referee_sender.set_robot_id()` 并填写 UI，`send()` 在偶拍空闲槽调用 `referee_sender.send()`，每次至多 1 帧
- `leg_down` 使用 `ChassisLeg::Config` 的质量、气弹簧推力、轮腿几何和额定轮速默认值；App 只绑定三组 PID 参数和 LQR 三张表，均定义在 `lib/robot/chassis_leg_lqr.hpp`（`LQR_FIT`、`LQR_POSTURE_FIT`、`LQR_R`，由 `agents/04-leg/lqr/gen_lqr_fit.py` 按同一 Q/R 生成）。默认轮速 65 rad/s、轮半径 0.06 m，对应直行目标上限 3.9 m/s
- `ChassisLeg::calc_lqr()` 先把遥控目标按 `wheel_work_speed × 轮半径` 等比缩小，再按四路额定力矩分配：十项贡献分运动四项（位移、前进速度、航向、旋转速度）与姿态六项（两腿角及角速度、俯仰及角速度），四路都在限幅内时原样下发；超限时姿态组优先，运动组乘同一个最大可行比例，全不可行时运动归零再整体缩小姿态组。腿摆上限由当拍轴向力矩折成两关节剩余余量，跳跃期间不做分配，下发仍由电机自身力矩限幅兜底
- 额定分配后按双轮净机械功率 `Σ Tw·ω ≤ power_ref` 修正（仿真 wheel_leg_lab `limit_wheel_power()` 的闭式投影）：未超或制动时原样下发；超出时取腿部映射 `M = D·C⁻¹`（C 为两轮对俯仰角速度、旋转速度的增益，D 为两腿稳姿态俯仰角速度增益与原腿增益之差、撤回的腿旋转速度项），`H = R轮 + Mᵀ·R腿·M`，轮力矩沿 `H⁻¹·ω` 投影到功率线；越过轮额定区间时从同比缩小点向投影点截取，两腿加 `M·轮缺口` 后再限到摆动额定区间。功率口径只计两轮机械功率，关节功率与铜耗由超电和缓冲能量承担；跳跃期间不修正。主机对照见 `agents/04-leg/power_alloc/`
- leg_up 发射机构按裁判剩余热量和冷却速率降频，`heat_delay`、`heat_gain`、`heat_reserve` 待上车整定；UI 的 yaw 相对角取 `yaw.angle`，世界 yaw/pitch 取 GimbalClassic 的 `angle_measure`，受击方向按“云台世界 yaw − yaw 电机角”换算底盘航向后锁存，底盘转速显示下板回传的 `down.wr`；下板通信、使能与倒地显示在 `yaw1`，6 个底盘电机状态显示在 `w1`~`w4`、`s1`~`s4`
