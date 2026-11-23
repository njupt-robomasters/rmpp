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
- 双 yaw 共用速度 PID 输出，各电机按自身原生单位接收同一数值；steer_sentry 的 DM4310 为 N·m、GM6020 为 A，参数待实车整定
- 17 mm 拨弹在 `update()` 内使用 STOP/FEED/REVERSE 状态机，失能、热量保护、零弹频或拨弹离线优先停止；拨弹独立于摩擦轮开关和在线状态
- 卡弹检测与退弹分别用 `block_elapsed`、`reverse_elapsed` 累加限幅后的 `dt`；上一周期拨弹 PID 输出绝对值达到 `shoot_pid_config->max_out`（饱和）并持续 `block_time` 后反向运行 `reverse_time`，阈值与输出同单位（随电机 `set_output()`：DJI 为 A、EL05 为 N·m）；纯 P 参数下堵转输出为 kp×目标转速，低于 max_out 时检测不到，需靠积分推到饱和。方向和速度跟随当前有符号弹频；STOP 清累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID
- 17 mm 拨弹先由状态机确定目标速度，再统一计算速度 PID；FEED 使用上一周期 `torque_ref` 检测卡弹，状态切换在下一周期改变目标速度

## 底盘离线反馈

- Motor 超时后 angle 清零，angle_multi 保留；轮腿接受离线关节角清零后的解算变化，双 yaw 的离线控制逻辑后续单独调整
- 舵轮仅在轮、舵均在线时允许对应轮输出；仅轮离线时，舵仍跟随速度逆解得到的目标角，零速度时保持目标角
- 速度正解排除离线位置：全向轮／麦轮按有效运动学行最小二乘求解，通常至少 3 轮；舵轮在无侧滑假设下按有效位置的 x/y 速度约束求解，至少 2 个不同位置。正常四轮舵轮解算与原均值算法等价，缺轮时重新求解平移与旋转，避免偏置
- 约束不足或秩亏时 vx/vy/vz/wr.measure 为 NaN；力控清空底盘速度 PID 并关闭轮输出，舵向控制继续；轮速控制仍按在线轮执行。恢复足够反馈后自动恢复解算
- 总牵引力反馈需要全部轮数据，缺失时 fx/fy/fz.measure 为 NaN；输出力求和与 PID 回算保持独立。两台哨兵对未知 wr 关闭云台转速前馈，现有 UI 数字字段显示 0，底盘测量本身保留 NaN
- leg_up 跟随模式下发 yaw.angle，小陀螺发 0；离线清零由 Motor::update() 统一完成

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
- 主机测试与 ARM 对象编译入口见 `agents/tasks/11-shooter_state/verify_gimbal_imu.py`；DualYaw 保持 ECD/IMU 模式

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
- leg_down 的 LQR 系数独立放在 `app/leg/leg_down/lqr_fit.hpp`，由 `agents/tasks/09-leg_lqr/gen_lqr_fit.py` 生成；`chassis.hpp` 只引用，当前参数与验证范围见任务 09
- 组件可独立使用
- 整合管理器引用外部实例

## 遥控器映射

- VT13 在 `update()` 内把遥控器与键鼠合成为操控量，app 优先读取：`is_enable`（N、S 挡）、`vx`/`vy`/`wr`、`yaw_speed`/`pitch_speed`、`bullet_speed`、`bullet_freq`、`is_aim`；原始量在 `rc`、`mouse`、`key`，供 fn、pause、校准组合键等 app 专用操作读取
- 平移与旋转为摇杆或拨轮乘满量程，叠加 W/S、A/D、Shift/Ctrl 键盘斜坡后限幅；按反向键时同向速度先清零再反向加速，松手按减速度回零；加减速度设 `INFINITY` 时一个周期到满量程或归零。斜坡用本类 dt，`DT_MIN`/`DT_MAX` 限幅
- 云台为摇杆叠加鼠标，无斜坡，限幅到满量程；pitch 抬头为负，与 Classic 云台一致。按住 trigger 时 pitch 摇杆改为弹频倍率（负值反转），否则鼠标左键按最高弹频；S 挡输出配置弹速；photo 或鼠标右键为自瞄
- 断联时原始量、键盘斜坡与操控量全部清零，挡位为 ERR
- 图传链路的裁判帧与遥控帧混在 VT13 串口：VT13 同一缓冲区按帧头分别校验，0xA5 帧只处理 0x0311（载荷至多 60 B），在串口接收中断内依序调用 `add_0311_callback()` 注册的回调，回调在启动前注册；0x0311 不影响 `is_connect`。机器人发往自定义终端的 0x0310 同样走图传串口，目前未提供发送接口；Referee 不再收发图传链路报文
- VT13 的 `Config` 公开且可修改，下次 `update()` 生效，如 steer_sentry 的 C、Z 键增减 `config.bullet_freq`；各 app 在实例处给出满量程、加减速度、弹速与弹频
- steer_sentry（VT13 接 USART6）与 jelly_sentry（VT13 接 USART1）只用 VT13，已移除 FSi6X；控制方式见下节“哨兵控制”。jelly_sentry 云台 pitch 取 `-vt13.pitch_speed`，保持迁移前摇杆上推目标增大的方向；步兵身份下 C、Z 键修改 `vt13.config.bullet_freq`
- FSi6X 只保留在 dummy、leg_down 断联回退与 `examples/rc`，在对应 `.cpp` 直接读取

## 哨兵控制

- steer_sentry 与 jelly_sentry 结构一致：`app.hpp` 提供 `rc_fn()`、`sentry_in_game()`、`sentry_active()`、`sentry_enable()`；chassis、gimbal、shooter 各有 `control_by_vt13()` 与 `control_by_auto()`，`*_loop()` 先设置使能，再按 `sentry_active()`（哨兵身份比赛中，或 VT13 N/S 挡按 fn 锁存测试，pause 清除）二选一，最后完成公共输入与 `update()`；自主时不叠加 VT13 操控量
- 使能：在线 VT13 的 C 挡优先失能；否则 VT13 N/S 挡，或裁判自检、倒计时、比赛中保持使能。发射机构另在 N 挡失能
- 底盘 VT13：分离模式，摇杆叠加键盘斜坡。底盘自主：导航速度为大 yaw 雷达系，经 `rotate(v, −gimbal.yaw2.measure)` 转到云台系并按 VT13 满量程限幅；flag 0 跟随模式不旋转（steer_sentry 未配跟随 PID，只平移），flag 1 分离模式小陀螺，平移 0→满量程时转速 `wr_max`→60 rpm；Nav 或任一 yaw 离线时停止
- 云台 VT13：摇杆叠加鼠标，R 键按下沿 `turn_yaw(180°)`，photo 或右键自瞄且识别到目标时跟随视觉角度；裁判自检阶段 yaw 叠加 180°/s 旋转。云台自主：5 m 内识别到目标且血量高于 120 时锁定视觉角度，脱锁 0.5 s 或低血量回到扫描（yaw 180°/s，pitch 在 IMU −22°~0° 往返 60°/s）；退出自主时复位为向下扫描。已删除全景相机（insta360）检查
- 发射 VT13：S 挡开摩擦轮，trigger 或左键给弹频，自瞄时需视觉开火许可，右键双击强制开火；C、Z 键调最高弹频（jelly_sentry 仅步兵身份）。发射自主：摩擦轮常开，视觉开火许可时按最高弹频。热量剩余量由裁判 `shooter_barrel_heat_limit − shooter_17mm_barrel_heat` 设置
- yaw 校零：VT13 C 挡按住 fn 与 photo 的按下沿，按大 yaw `angle_multi` 更新 offset 并写入 KvStore；steer_sentry 沿用改名前的 KvStore 标记 `"sentry"`，保留已标定零位
- 操控参数和跨周期状态使用文件内 `static`

## 持有与更新

- 设备由 app 定义；只服务一个机构的设备（电机）由该机构持有并在其 `update()` 中更新
- 被多处使用或与机构业务无关的设备（姿态 `Attitude`、超电 `SuperCap`）由 app 调用 `update()`，结果再交给机构：每周期都要刷新的测量经机构 `update(...)` 参数传入（如底盘和 Classic 云台 `update(q, gyro)`、DualYaw `update(yaw, pitch)`），其余输入用 `set_*`（如 `set_power_ref()`、`set_gimbal_yaw()`、`set_leg_length()`）
- app 先更新共享设备再更新机构：如 `super_cap.update()` → `chassis.set_power_ref(super_cap.power_chassis_ref)` → `chassis.update()`
- 发送（`send()`、`write_can_frame()` 组帧）始终由 app 安排
- 模块间不互相引用状态
- 多模块共用判断放在 `app.hpp`

## jelly_sentry 超级电容

- 已装超电，CAN1，沿用 `SuperCap` 默认命令 ID 0x061、反馈 ID 0x051；复用 dummy 的协议和虚拟缓冲算法，整车输入使用真实裁判功率上限与缓冲能量
- `chassis_loop()` 每周期设置模式、裁判输入并更新超电，再把 `power_chassis_ref` 交给四轮力控底盘；发送在 `app.cpp::send()` 独立按 10 拍分频，目标 100 Hz，OFF、底盘失能和超电离线时持续发送
- 模式优先级：裁判离线或在线 VT13 C 挡 → OFF；VT13 N/S 挡、或正赛且机器人 ID 为 7/107 → BOOST；自检/倒计时 → CHARGE；其余 OFF。BOOST 工况下用电池侧余量充电，负载升高时允许电容补偿。比赛自主运行可在遥控断联后继续使用超电，仍受裁判连接状态约束
- 裁判连接超时由现有 `Referee` 的 1 s 检测决定，超电转 OFF 后底盘继续使用最后功率上限与缓冲能量；上电未收到裁判时为默认 120 W/60 J，超电 OFF
- 超电实测功率覆盖范围按实车供电支路核对，四轮模型只估计轮电机功率；UI 的 CAP 弧长和整数百分比均取自 `SuperCap::energy_ratio()`，按（当前电容电压 / 满电电压）² 计算，满电默认 23.85 V，离线显示 0
- CAP、RPM、BULLET、SHOOT 四段仪表圆弧始终为绿色；整车图中的超电状态框按超电状态着色
- 实施与验证记录见 `agents/tasks/06-super_cap_capacitance/README.md`，硬件联调由用户审计后决定

## jelly_sentry 双传送带驱动

- 在线 VT13 的 C 挡使整车失能，优先于比赛阶段的自动使能；N 挡只允许底盘和云台使能，摩擦轮与拨弹目标均归零，发射机构失能
- 用户说明 rub1/rub2 为工业传送带转动机构，工程沿用摩擦轮命名。VT13 在线时只按 VT13 手动控制：S 挡运行两路电机（`vt13.bullet_speed`），拨盘取 `vt13.bullet_freq`（trigger 时 pitch 摇杆乘最高弹频，负值反转；否则鼠标左键按最高弹频），人工自瞄需视觉开火许可，右键双击强制开火除外；自主时双传送带常开、视觉开火许可时按最高弹频
- 两个 P630 由 app 定义为 `Vesc`，绑定 CAN2，节点 ID 沿用 `0x21/0x22`，共用现有 VESC 扩展帧收发实现
- `Shooter17mm::update()` 通过 `set_speed()` 设置输出轴目标角速度（rad/s），由 Vesc 换算为 ERPM，使用电调内部速度环
- 整车每1 ms发送两路VESC指令，使能时速度扩展帧ID为0x321/0x322、4字节大端ERPM，失能时发送零电流帧
- 当前显式保存迁移前Vesc参数：20 A、20 Nm、reduction为7（包含极对数）；按合并前实机文件保留rub1方向true、rub2默认false，同步修正速度指令与反馈方向，实际极对数待核对

## jelly_sentry EL05

- 拨弹 EL05 绑定 CAN2，按 old 分支移植私有扩展帧协议，电机节点0x12、主机节点0x22；使能ID 0x03002212、停止ID 0x04002212、无故障运行反馈ID 0x02801222。两路pitch仍用原标准帧协议
- 电机端配套参数：CAN_ID=18、CAN_MASTER=34、protocol_1=0、CAN_TIMEOUT=20000（约1秒）；此前上位机写入记录为protocol_1=2，需用户核对并改为0后断电重启，主控不在线改参
- 整车shoot实例控制计算保持1ms，EL05按shoot_send_period_ms发送（默认2ms、500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答
- 按用户实测反向要求，shoot在shooter.hpp中显式配置is_invert=true，同步反向力矩指令和位置/速度/力矩反馈；三路方向可分别观察rub1.config.is_invert、rub2.config.is_invert和shoot.config.is_invert
- 卡滞参数在shooter.hpp的shooter配置中显式设置：block_time=0.2 s、reverse_time=0.25 s；检测阈值为拨弹PID的5 N·m输出限幅，现有纯P参数（kp≈0.318 N·m/(rad/s)）在约22.5 Hz以下堵转不饱和，待加积分整定

## jelly_sentry 上游接口适配

- `GimbalFold` 位于 `lib/robot/gimbal/gimbal_fold.*`，目前仅 jelly_sentry 使用；四轴控制在独立的 `update()` 中执行。app 每周期更新 attitude，并通过 `gimbal.update(attitude.yaw, attitude.pitch)` 传入姿态，yaw1 零点由 app 经 KvStore 持久化
- 大 yaw、pitch1、pitch2 经电机原生 `set_output()` 接收 Nm，小 yaw 7025 接收 A；折叠云台保留电机限幅后的 PID 跟踪及重力补偿，独立于上游普通云台的简化策略
- 底盘由 app 更新超电后传入 `power_chassis_ref`
- 受击方向显示与实测弹速读取裁判公开只读的 `hurt_data`、`hurt_sequence`（收到 0x0206 递增）、`shoot_data`；受击时按装甲 ID 换算到 IMU 参考系锁存 1 s。两台哨兵与 leg_up 在 `referee_loop()` 直接填写 `ui` 的公开字段，再调用 `ui.update(referee_sender)`
- 整车图 ADD 时全白，中心弧固定绿色，不再显示裁判断连。槽位按关节命名：`yaw1` 在轮腿上为下板、在双 yaw 哨兵上为大 yaw，`yaw2` 为小 yaw，`pitch1`、`pitch2` 为俯仰关节，未使用的槽位保持白色（steer_sentry 无 `pitch2`）。电机状态由电机对象提供，各车 `referee.cpp` 配置过热阈值（默认 80 ℃）并选色；离线粉、主动失能白、过热橙、达妙错误黄、正常绿。轮腿的 6 个底盘电机状态由下板状态帧回传：`yaw1` 显示下板断联粉、未使能白、倒地青、正常绿，`w1`~`w4` 为左前、左后、右后、右前关节，`s1`、`s2` 为左轮，`s3`、`s4` 为右轮。摩擦轮上排对应左侧、下排对应右侧；图上左列为 1 级、右列为 2 级，单级同侧共用状态。轮腿无自瞄连接源，`aim` 保持白色。超电离线粉、OFF 白、CHARGE 青、BOOST 绿
- 超电处于 BOOST 但电压达到或低于配置的放电补偿截止值时显示青色，高于截止值时显示绿色
- 独立 `examples-motor` 台架示例通过 SWD 查看 `motor_status_color`：断连粉、失能或零目标白、过热橙；其余状态下转速偏差超过 5 rpm 为青，达标为绿。该观察量不参与电机控制或实车 HUD

## jelly_sentry 导航与 pitch1

- CDC1 留给 Aim，CDC2 接 Nav 的 17 字节导航下行帧；主循环先执行 nav.update()，再执行 chassis_loop()
- 导航按大 yaw 雷达系速度接入，经负小 yaw 角转换到云台系；flag=0 随 IMU 头，flag=1 不随头并使用小陀螺。人工底盘输入优先接管，导航超时或 yaw 反馈离线停止自主运动
- pitch1 当前全程失能，GimbalFold 的总使能接口和每次 update 固定关闭该轴，清 PID 并写零输出，保留反馈与失能帧发送；其余三轴正常控制
- 协议、坐标公式、flag 和离线处理详见 `minipc.md`；验证见 `../tasks/15-minipc_nav/README.md`

## jelly_sentry CAN 发送调度

- 总线分配：CAN2 为 pitch1、pitch2、EL05 和两个 P630，CAN3 为小 yaw 7025；协议、ID 和发送周期保留原值
- 大yaw DM4310使用CAN1，当前命令ID 0x11、反馈ID 0x10；CAN2自动重发当前开启，用户生成的fdcan.c与.ioc均为ENABLE，发送FIFO模式保留
- 小 yaw 中位原始角度由用户实测为 5.31959 rad，`yaw2.config.offset` 已按该值配置；更新固件后核对中位 `yaw2.angle` 接近 0
- EmberProbe 实测 IMU 航向与大小 yaw 合成角方向相反；按用户确认将大 yaw 设为 `is_invert=true`、小 yaw 设为 `false`，同步翻转位置、速度反馈和输出，使合成角与 IMU 航向方向一致，保留原始 offset；待更新固件后低输出复核

- 整车控制计算保持1kHz；pitch1、pitch2和yaw2的发送周期可写变量放在gimbal.hpp各电机实例旁，EL05的shoot_send_period_ms放在shooter.hpp的shoot实例旁
- 默认周期：pitch1_send_period_ms=4（250Hz）、pitch2_send_period_ms=1（1000Hz）、yaw2_send_period_ms=2（500Hz，A1/94各250Hz）、shoot_send_period_ms=2（500Hz）
- 当前整车启用底盘、大小yaw、pitch2、EL05、双传送带及超电控制帧发送；pitch1 保留发送且始终发送失能帧。EL05默认2 ms周期调用，合并后需用户更新固件核对在线状态
- 各变量为inline volatile uint32_t，运行时通过调试器写入，下一个发送判断使用新值，不清零已有计数。计数器保留在cpp，不作为调频入口；周期0等效每主循环发送，超过100ms可能触发主控反馈离线，7025建议周期至少2ms
- 失能、使能和清错帧替换当次发送，不额外增加发送频率；CAN2 和 CAN3 分别核算负载，P630 扩展帧和状态上报按实际电调设置统计
- 频率是按1ms主循环计算的目标值，真实周期、反馈频率与7025断联是否改善仍需上板测量

## 轮腿上下板通信

- 上下板 Can(1) 经滑环连接，与下板全部电机、上板管理的超电共用 CAN1；两板通过 `../leg_protocol.hpp` 引用共享文件 `app/leg/leg_protocol.hpp`，包含按发送方命名的 `UP_ID = 0x02`、`DOWN_ID = 0x03` 和报文布局。上板控制帧 8 字节（使能、跳跃、速度、yaw、腿长），500 Hz；下板状态帧 4 字节（底盘倒地、下板使能、实测航向角速度、6 个电机状态），250 Hz
- 共享协议文件统一定义 `LegUpFrame`、`LegDownFrame`、物理单位、`pack()` 与 `unpack()`；`Raw` 位域与解码后成员同序，编解码在 `pack()`、`unpack()` 内用局部 `Raw` 完成，不保存原始帧；位域按 arm-none-eabi-gcc 布局，主机 MinGW 测试加 `-mno-ms-bitfields`。解包先检查 ID 和长度，失败保持原数据。腿长为连续数值 `leg_length`（m），以 9 位无符号毫米数传输，范围 0..0.511 m；协议中没有腿长档位表。`LegProtocol<TxFrame, RxFrame>` 封装发送帧、接收帧、在线状态与超时：构造时向 Can 注册回调，`update()` 超时清接收数据，`send()` 组帧发送；上板用 `LegUpProtocol`、下板用 `LegDownProtocol`，两板 `chassis_loop()` 直接读写 `leg_protocol` 字段，发送仍按 `app.cpp` 的拍数调用 `leg_protocol.send()`；回归测试检查共享文件引用与编解码
- 上板在 VT13 使能且下板在线时，首次按下 Pause 使能底盘，之后按 Pause 按 0.14 → 0.22 → 0.33 → 0.22 → 0.14 m 往返切换腿长。Z/X/C 各自在按下沿直选低/中/高档；选低档后 Pause 向上切，选高档后向下切，选中档保留原方向。E 或 Fn 任一按住时下发跳跃位，由下板检测上升沿。失能或下板断联时清使能锁存、复位低档，恢复后重新等待 Pause；每周期记录 Pause 和三个腿长键的前值
- `vt13.vx`（右摇杆竖直叠加 W/S）为云台前向速度，`vt13.wr`（拨轮叠加 Shift/Ctrl）为手动旋转。上板控制帧的 `wr` 为航向角速度目标：`vt13.wr` 非 0 时发手动角速度、`yaw` 发 0（小陀螺，下板不追踪航向）；为 0 时发遥控器云台 `yaw_speed` 作跟随前馈（与云台角度外环解耦，自瞄时前馈为 0），`yaw` 发 `yaw.angle` 相对航向；yaw 离线时角度发 0。上板直接保存解包后的 `down`，pause 分支使用 `down.is_fallen`，即下板 `chassis.rescue.is_fallen`（倒地锁存，站起完成、失能或跳跃时清除），断联 0.1 s 清空反馈。在线状态命名为 `up_is_connect`、`down_is_connect`
- 下板执行策略：FSi6X 在线时优先接管，避免上板协议错误导致误动作；否则透传上板跳跃与腿长，前向速度按 `cos(yaw)` 投影，上板断联 0.1 s（`LEG_TIMEOUT`）后按已清零的指令失能。`ChassisLeg` 的 `wr` 为航向角速度目标，`gimbal_yaw` 恒按 `Config::yaw_err_max` 对称限幅（默认 ±1.25 rad）后作航向误差；小陀螺时上板发 0，误差项为 0，仅跟踪手动角速度。FSi6X 路径保留拨杆操作，云台航向设为 0，保留 LQR 速度与位移反馈
- 自救授权仍由下板决定：控制帧删除自救位，上板控制时 `control_by_up()` 固定 `set_allow_rescue(true)`，FSi6X 控制时保持 SWB 下挡授权；底盘使能、倒地且已授权空闲时启动自救，站起完成、失能或跳跃时清除锁存
- 超电接上板 Can(1)，每周期读取裁判功率上限和缓冲能量并更新，指令按 100 Hz 发送。模式跟随 VT13 整车使能：失能 OFF，使能 BOOST。
- 上板控制帧：bit0 使能，bit1 跳跃，bit2..14 前向速度，bit15..30 航向角速度目标，bit31..43 yaw，前三个运动量分别按 0.001 m/s、0.001 rad/s、0.001 rad 编码；bit44..52 腿长，bit53..63 保留，发送时清零。下板状态帧：bit0 倒地，bit1 使能，bit2..17 实测航向角速度（int16，0.001 rad/s），bit18..29 为左前、左后、右后、右前、左轮、右轮的 2 位状态（0 正常、1 过热、2 达妙错误、3 离线），bit30..31 保留。
- 下板 Can(1) 每拍 3 帧，匹配 3 个发送邮箱：偶拍左腿两关节与双轮 0x200，奇拍右腿两关节加第 3 帧，第 3 帧由 0x003 状态帧与 `referee_relay.send()` 轮流使用，各 250 Hz
- 裁判系统接下板 USART1（115200），`app.hpp` 的 `RefereeRelay referee_relay` 原样转发字节流：CAN ID 与 leg_protocol 一样按发送板命名，0x00 为下板发送、0x01 为上板发送，类内按本板视角写作 `tx_id`、`rx_id`（与 Referee 对应）。串口收到的字节切成至多 8 字节的 0x00 帧，每次 `send()` 发 1 帧，容量 2 kB/s，约为 2027 协议常规链路定频报文的 2 倍；收到的 0x01 帧载荷拼接后在串口空闲时一次发出。缓冲区 `to_can` 1024 B（1 Hz 定频报文与触发报文叠加的最坏突发约 500 B）、`to_uart` 256 B（两次 `send()` 间至多积压 32 B），满时丢弃并计入 `to_can_drop`、`to_uart_drop`。上板用 `Referee({.can = Can(1)})` 接收（默认 `rx_id = 0`）、`RefereeSender({.can = Can(1)})` 发送（默认 `tx_id = 1`）；leg_up 的 `referee_loop()` 在 `referee.update()` 后调用 `referee_sender.set_robot_id()` 并填写 UI，`send()` 在偶拍空闲槽调用 `referee_sender.send()`，每次至多 1 帧
- `ChassisLeg::calc_lqr()` 直接输出左右轮力矩与两腿摆动力矩，轮电机保留自身力矩限幅。
- leg_up 发射机构按裁判热量设置剩余热量，`heat_protect = 80` 待整定；UI 的 yaw 相对角取 `yaw.angle`，世界 yaw/pitch 取 GimbalClassic 的 `angle_measure`，受击方向按“云台世界 yaw − yaw 电机角”换算底盘航向后锁存，底盘转速显示下板回传的 `down.wr`；下板通信、使能与倒地显示在 `yaw1`，6 个底盘电机状态显示在 `w1`~`w4`、`s1`~`s4`
