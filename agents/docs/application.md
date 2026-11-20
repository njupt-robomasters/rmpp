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
- 卡弹检测与退弹分别用 `block_elapsed`、`reverse_elapsed` 累加限幅后的 `dt`；目标力矩绝对值连续达到 `config.block_torque` 并持续 `block_time` 后反向运行 `reverse_time`，阈值为0时沿用电机最大力矩。方向和速度跟随当前有符号弹频；STOP 清累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID
- 17 mm 拨弹先由状态机确定目标速度，再统一计算速度 PID；FEED 使用上一周期 `torque_ref` 检测卡弹，状态切换在下一周期改变目标速度

## 底盘离线反馈

- Motor 超时后 angle 清零，angle_multi 保留；轮腿接受离线关节角清零后的解算变化，双 yaw 的离线控制逻辑后续单独调整
- 舵轮仅在轮、舵均在线时允许对应轮输出；仅轮离线时，舵仍跟随速度逆解得到的目标角，零速度时保持目标角
- 速度正解排除离线位置：全向轮／麦轮按有效运动学行最小二乘求解，通常至少 3 轮；舵轮在无侧滑假设下按有效位置的 x/y 速度约束求解，至少 2 个不同位置。正常四轮舵轮解算与原均值算法等价，缺轮时重新求解平移与旋转，避免偏置
- 约束不足或秩亏时 vx/vy/vz/wr.measure 为 NaN；力控清空底盘速度 PID 并关闭轮输出，舵向控制继续；轮速控制仍按在线轮执行。恢复足够反馈后自动恢复解算
- 总牵引力反馈需要全部轮数据，缺失时 fx/fy/fz.measure 为 NaN；输出力求和与 PID 回算保持独立。sentry 对未知 wr 关闭云台转速前馈，现有 UI 数字字段显示 0，底盘测量本身保留 NaN
- leg_up 直接下发 yaw.angle，离线清零由 Motor::update() 统一完成

## GimbalClassic 控制与观测

- 机构为底盘→yaw→pitch，IMU 固连 pitch 随动部分；`update(q, gyro)` 接收云台到世界的单位四元数和完成安装修正的云台系角速度（X 沿枪口、Y 沿 pitch 正轴）。pitch 编码器零位对应两级坐标系对齐
- 内环：`pitch_joint = AngleAxisf(motor.pitch.angle, UnitY())` 把 gyro 旋转到 yaw 随动系，Z/Y 分量为 yaw、pitch 电机轴的绝对角速度；yaw 再减 `chassis_wr` 换算为相对底盘转速，pitch 保留绝对角速度，作为各自的 `speed_measure`；电机编码器速度只供观察
- 外环：`angle_ref/angle_measure` 为枪口世界 yaw/pitch（ZYX 欧拉角）。ZYX 的 yaw/pitch 完整描述枪口指向，roll 只是绕枪口自旋，两轴机构无法也无需控制，因此外环直接对欧拉角误差闭环（yaw 误差经 `Angle` 回绕），省去目标四元数与 roll 参考
- 外环加速度规划：`Config` 的 `yaw_max_accel`、`pitch_max_accel` 默认 `INFINITY`（原样传入误差，等效禁用规划）；设为有限正数时，在线性区阈值 `e0 = max_accel / kp²` 内保持原误差，超出后经 `plan_angle_error()` 折算为 `±√(e0·(2|e| − e0))` 再送入角度 PID，使比例项沿恒减速度平方根制动曲线 `±√(2·max_accel·(|e| − e0/2))` 平滑收敛，在 `e0` 处值与斜率连续，消除阶跃目标与外力大扰动下的制动饱和超调
- `set_speed()` 为电机轴速度，直接作为 `speed_ref` 的前馈；外环开启时同时积分到世界角度目标（底盘水平时两者一致，倾斜时的差异由外环修正）。外环输出的世界 yaw/pitch 角速率修正合成 `ψ̇·Z + θ̇·(−sinψ, cosψ, 0)` 后经 `pitch_joint * q⁻¹` 换算到电机轴再叠加，底盘倾斜时消除轴间耦合，roll 不引入稳态偏差。pitch 接近 ±90 度时世界 yaw 奇异，实际实例由 pitch 保护范围避开
- `use_angle_loop = false` 只保留速度内环；角度目标跟随实测，`set_angle()` 仅清零速度指令
- 使能与在线判断集中为 `can_pitch = is_enable && pitch 在线`、`can_yaw = can_pitch && yaw 在线`（yaw 换算依赖 pitch 关节角）。外环由私有成员 `update_angle_loop()` 直接读写两轴状态和角度 PID：不可控、关闭外环或首次 `update()` 时目标跟随实测并清空角度 PID，恢复后自然从当前姿态开始，无需上周期状态；速度内环不可控时清零目标、清空 PID、输出 0
- `angle_ref/angle_measure`、`speed_ref/speed_measure` 即两级闭环的最终目标与反馈：`speed_ref` = 电机轴绝对速度指令 + 换算后的外环修正；yaw 目标与 IMU 反馈同时减 `chassis_wr`，统一为相对底盘的电机转速。速度误差为 `speed_ref − speed_measure`，两处底盘转速相消，保持绝对速度指令与世界角度闭环
- pitch 目标限位：外环每周期取 `offset = pitch.angle_measure − motor.pitch.angle`，把编码器限位平移为世界 pitch 范围 `[pitch_min + offset, pitch_max + offset]`，钳制速度积分后的目标，`set_angle()` 的越界目标在下一次 `update()` 同样钳制；目标停在边界后不再累积，避免误差经 `Angle` 回绕后反转到另一侧。目标被钳住且 `pitch_speed` 向外时从 `speed_ref` 撤掉该前馈，否则平衡点会冲出限位约 `pitch_speed / kp`；向内指令保留前馈。底盘倾斜时世界 pitch 与关节角的二阶非线性由每周期刷新的 offset 吸收
- pitch 输出保护兜底超调：编码器 `<= pitch_min` 时禁止负输出、`>= pitch_max` 时禁止正输出，截断后 `track(0)` 回算积分，保留回程方向；只开速度环时仅有此保护
- 主机测试与 ARM 对象编译入口见 `agents/tasks/11-shooter_state/verify_gimbal_imu.py`；DualYaw 保持 ECD/IMU 模式

## 启动与循环

- CubeMX 初始化后依次执行 `bsp_init()`、`app_main()`
- `app.cpp` 分为 `setup()`、`loop()`、`app_main()`：`setup()` 放一次性初始化；`loop()` 依次轮询各设备与模块；`app_main()` 先调用 `setup()`，再用循环内的局部 `static Timer app_timer` 按 1 kHz 调度 `loop()`，并用循环内的局部 `static Monitor app_monitor` 统计频率与占用（SWD 观察）
- 计时器、分频计数等跨周期状态通常放在文件作用域 `static`；主循环的 `app_timer` 放在 `while (true)` 内“1 kHz 更新”注释后，`app_monitor` 放在“负载监控”注释后，均保留 `static`
- 低于 1 kHz 的任务在 `loop()` 内用 `<用途>_timer` 分频后提前返回，如 cdc 示例的 `print_timer`
- 自己调用 `Pid` 的 app 函数用 `update_timer` 计一次周期并限幅后传给各 PID
- dummy 和 leg 用 `uint32_t` 计数递增、取余分频，无手动清零：dummy 的 yaw 每 2 次循环发送，超电每 10 次循环发送，模拟裁判每 100 次循环结算；leg_down 左右关节交替发送，每个关节 500 Hz，底盘只在发送左腿的一拍调用 `chassis_loop()`（500 Hz），保证计算时两侧关节反馈都已刷新，`attitude.update()` 仍在 `loop()` 每周期执行；leg 两板的发送计数 `send_cnt` 按用户要求为 `loop()` 内 `static`
- 使用普通 loop，不引入操作系统或整车包装类

## 文件组织

- 公共内容放在 `app.hpp`
- 各模块在同名 `.hpp` 声明、`.cpp` 实现
- 文件名不加 `_loop` 后缀
- 配置和实例跟随所属模块
- leg_down 的 LQR 系数独立放在 `app/leg/leg_down/lqr_fit.hpp`，由 `agents/tasks/09-leg_lqr/gen_lqr_fit.py` 生成；`chassis.hpp` 只引用，当前参数与验证范围见任务 09
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

## jelly_sentry 超级电容

- 已装超电，CAN1，沿用 `SuperCap` 默认命令 ID 0x061、反馈 ID 0x051；复用 dummy 的协议和虚拟缓冲算法，整车输入使用真实裁判功率上限与缓冲能量
- `chassis_loop()` 每周期设置模式、裁判输入并更新超电，再把 `power_chassis_ref` 交给四轮力控底盘；发送在 `app.cpp::send_can_cmd()` 独立按 10 个主循环分频，目标 100 Hz，OFF、底盘失能和超电离线时持续发送
- 模式优先级：裁判离线或任一在线遥控器显式 OFF → OFF；在线 VT13 N/S 挡、或正赛且机器人 ID 为 7/107 → BOOST；自检/倒计时 → CHARGE；其余按人工请求选择 BOOST、CHARGE，均无请求时 OFF
- 人工映射：FSi6X SWC 上/中/下为 OFF/CHARGE/BOOST，VT13 C/N/S 为 OFF/BOOST/BOOST。BOOST 工况下用电池侧余量充电，负载升高时允许电容补偿。比赛自主运行可在遥控断联后继续使用超电，仍受裁判连接状态约束
- 裁判连接超时由现有 `Referee` 的 1 s 检测决定，超电转 OFF 后底盘继续使用最后功率上限与缓冲能量；上电未收到裁判时为默认 120 W/60 J，超电 OFF
- 超电实测功率覆盖范围按实车供电支路核对，四轮模型只估计轮电机功率；首轮保留现有参数和 `ui.cap=0`，能量显示待确定满电电压与可用能量范围后接入
- 实施与验证记录见 `agents/tasks/06-super_cap_capacitance/README.md`，硬件联调由用户审计后决定

## jelly_sentry 双传送带驱动

- 在线 VT13 的 C 挡使整车失能，优先于比赛阶段的自动使能；N 挡只允许底盘和云台使能，摩擦轮与拨弹目标均归零，发射机构失能
- 用户说明 rub1/rub2 为工业传送带转动机构，工程沿用摩擦轮命名。在线 VT13 的 S 挡运行两路电机，按住扳机后，左摇杆上下按比例给拨盘速度，负值反转；松开扳机或摇杆回中，拨盘目标归零
- 两个 P630 由 app 定义为 `Vesc`，绑定 CAN2，节点 ID 沿用 `0x21/0x22`，共用现有 VESC 扩展帧收发实现
- `Shooter17mm::update()` 通过 `set_speed()` 设置输出轴目标角速度（rad/s），由 Vesc 换算为 ERPM，使用电调内部速度环
- 整车每1 ms发送两路VESC指令，使能时速度扩展帧ID为0x321/0x322、4字节大端ERPM，失能时发送零电流帧
- 当前显式保存迁移前Vesc参数：20 A、20 Nm、reduction为7（包含极对数）；按合并前实机文件保留rub1方向true、rub2默认false，同步修正速度指令与反馈方向，实际极对数待核对
- 2026-11-12：整车 app 及三个发射调试入口的相关对象文件经 Arm GCC 局部编译通过，无新增编译警告；未完整链接、烧录或硬件验证。`git diff --check` 通过，本机未找到 clang-format，未完成工具格式检查

## jelly_sentry EL05

- 拨弹 EL05 绑定 CAN2，按 old 分支移植私有扩展帧协议，电机节点0x12、主机节点0x22；使能ID 0x03002212、停止ID 0x04002212、无故障运行反馈ID 0x02801222。两路pitch仍用原标准帧协议
- 电机端配套参数：CAN_ID=18、CAN_MASTER=34、protocol_1=0、CAN_TIMEOUT=20000（约1秒）；此前上位机写入记录为protocol_1=2，需用户核对并改为0后断电重启，主控不在线改参
- 整车和发射调试入口共用shoot实例，控制计算保持1ms，EL05按shoot_send_period_ms发送（默认2ms、500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答
- 按用户实测反向要求，shoot在shooter.hpp中显式配置is_invert=true，同步反向力矩指令和位置/速度/力矩反馈；三路方向可分别观察rub1.config.is_invert、rub2.config.is_invert和shoot.config.is_invert
- 卡滞参数在shooter.hpp的shooter配置中显式设置：block_torque=5 N·m、block_time=0.2 s、reverse_time=0.25 s；检测阈值独立于电机与PID的5 N·m输出限幅，修改配置后重新构建并更新固件

## jelly_sentry 上游接口适配

- `GimbalFold` 位于 `app/jelly_sentry/gimbal_fold.*`，由折叠哨兵及其云台调试目标编译；四轴控制在独立的 `update()` 中执行。app 每周期更新 attitude，并通过 `gimbal.update(attitude.yaw, attitude.pitch)` 传入姿态，yaw1 零点由 app 经 KvStore 持久化
- 大 yaw、pitch1、pitch2 经电机原生 `set_output()` 接收 Nm，小 yaw 7025 接收 A；折叠云台保留电机限幅后的 PID 跟踪及重力补偿，独立于上游普通云台的简化策略
- 单轴与拨弹测试显式限幅 dt 并传给 PID；底盘由 app 更新超电后传入 `power_chassis_ref`
- 受击方向显示与实测弹速继续读取裁判 `hurt_data`、`hurt_sequence`、`shoot_data`，这些已有字段为公开只读输出

## jelly_sentry 大小 yaw 配合

- 2026-11-21 按用户要求参考 Joushua233/rmpp 的 old 分支 `e7bbcb7`，采用 `rmpp/lib/module/gimbal/Gimbal_DualYaw.cpp` 的持续追随分配，移除45°/35°窗口及相关配置和状态
- 大 yaw 编码器目标为 `yaw.ecd.ref`；小 yaw 目标为 `Angle(yaw.ecd.ref - yaw1.measure)`，按现有 ±40° 软件限位钳制。大轴跟上总目标后，小轴自然回中
- 大 yaw 速度前馈为 `yaw_speed - chassis_wr`；小 yaw 为 `yaw_speed - chassis_wr - motor.yaw1.speed`，小轴目标超出限位时撤掉前馈。大轴使用自身编码器速度闭环，保留现有PID参数和720°/s位置环输出上限
- 2026-11-21 按用户要求将 EmberProbe 实时调参值写回 `gimbal.hpp`：大轴速度环kp=0.5、ki=0.1，位置环kp=10；小轴速度环kp=2、ki=0.1，位置环kp=20，均为SI单位；其余PID字段沿用现有值
- `yaw.imu.ref` 持续保留最终头部目标，不再在 update 末尾被大轴目标覆盖；`yaw1.ref` 为分配后的大轴编码器目标，`yaw2.ref` 为限幅后的小轴目标
- 离线轴保持最后测量、输出零；IMU模式小轴离线时使用保持的小轴角度修正大轴目标，其余离线恢复、失能及模式转换沿用现有实现。底盘角速度非有限时关闭该项前馈，保留姿态闭环
- 该分配采用两根 yaw 轴平行、存在位置偏置的姿态模型；相机/枪口随偏心轴运动造成的视差由视觉外参与距离处理。pitch1 继续固定失能，俯仰控制和 CDC 协议保持原有接口
- old分支来源为 [Gimbal_DualYaw.cpp](https://github.com/Joushua233/rmpp/blob/e7bbcb74a9acb41d673ff5b8c167fbd4f1bc3ab0/rmpp/lib/module/gimbal/Gimbal_DualYaw.cpp)。机械耦合与超调表现由用户审计后上板核对

## jelly_sentry 调试入口清理

- 2026-11-21 按用户要求删除 `app/jelly_sentry_debug/` 全部源码及11个调试预设；保留整车 `jelly_sentry` 预设，预设生成脚本按剩余应用目录生成列表

## jelly_sentry 导航与 pitch1

- CDC1 接 Aim 的 21 字节上行/18 字节下行自瞄帧，CDC2 接 Nav 的 17 字节导航下行帧；主循环先向 Aim 传入头部姿态和裁判实测弹速，再更新 Nav，最后执行底盘、云台及发射控制
- Aim 每 5 ms 上传最终 IMU 头姿态，默认 200 ms 无有效视觉帧时清空检测和开火许可；`aim.vision` 为最终 IMU 头世界系目标，内部弧度，协议度，上正 pitch 在边界反号
- 导航按大 yaw 雷达系速度接入，经负小 yaw 角转换到云台系；flag=0 随 IMU 头，flag=1 不随头并使用小陀螺。人工底盘输入优先接管，导航超时或 yaw 反馈离线停止自主运动
- pitch1 当前全程失能，GimbalFold 的总使能接口和每次 update 固定关闭该轴，清 PID 并写零输出，保留反馈与失能帧发送；其余三轴正常控制
- 协议、坐标公式、flag 和离线处理详见 `minipc.md`；验证见 `../tasks/13-minipc_nav/README.md`

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
- gimbal单轴调试共用pitch2周期；gimbal_motor通过send_period_ms引用当前测试电机的周期变量；shooter与shooter_shoot共用shoot周期。变量仅影响CAN发送，反馈按电机应答而定，不改变PID计算周期

## 轮腿上下板通信

- 上下板 Can(1) 经滑环连接，与下板全部电机、上板管理的超电共用 CAN1；两板通过 `../leg_protocol.hpp` 引用共享文件 `app/leg/leg_protocol.hpp`，包含按发送方命名的 `UP_ID = 0x02`、`DOWN_ID = 0x03` 和报文布局。上板控制帧 8 字节（使能、自救授权、跳跃、速度、yaw、腿长、功率目标），500 Hz；下板状态帧 3 字节（底盘倒地、实测航向角速度），500 Hz
- 共享协议文件统一定义 `LegUpFrame`、`LegDownFrame`、物理单位、`pack()` 与 `unpack()`；`Raw` 位域与解码后成员同序，编解码在 `pack()`、`unpack()` 内用局部 `Raw` 完成，不保存原始帧；位域按 arm-none-eabi-gcc 布局，主机 MinGW 测试加 `-mno-ms-bitfields`。解包先检查 ID 和长度，失败保持原数据。功率目标 11 位 0.25 W/LSB，范围 0..511.5 W，2047 表示不限功率。腿长为连续数值 `leg_length`（m），以 9 位无符号毫米数传输，范围 0..0.511 m；协议中没有腿长档位表。两板 `chassis.cpp` 保存协议结构体并调用编解码，负责控制与在线状态，发送仍在 `app.cpp` 的 `loop()` 分拍安排；回归测试检查共享文件引用与编解码
- 暂不适配键鼠：上板只读 VT13，VT13 与下板均在线时 N、S 挡使能，默认低腿长 0.14 m。下板报告倒地时按 pause 授权自救，授权保持；站立时每次按 pause 按 0.14 → 0.22 → 0.33 → 0.14 m 切换。fn 直接透传，由下板检测上升沿。失能或下板断联清授权、复位低腿长；下板断联期间发送失能帧，恢复状态帧后按当前遥控挡位恢复使能。pause 持续记录前值
- 右摇杆竖直为云台前向速度，拨轮为手动旋转；yaw 在线时默认跟随，离线角度发 0。上板直接保存解包后的 `down`，云台使用 `down.wr`，pause 分支使用 `down.is_fallen`，即下板 `chassis.rescue.is_fallen`（倒地锁存，站起完成、失能或跳跃时清除），断联 0.1 s 清空反馈。在线状态命名为 `up_is_connect`、`down_is_connect`
- 下板执行策略：上板在线时透传自救、跳跃和腿长，前向速度按 `cos(yaw)` 投影；`ChassisLeg` 持有 follow PID，使用本类 dt，使能且无手动旋转时跟随经 `set_gimbal_yaw()` 设置的云台相对航向（为 0 时 P 输出 0），手动旋转或失能时清 PID；断联 0.1 s（`LEG_TIMEOUT`）回退 FSi6X 并将云台航向设为 0，保留 FSi6X 原有拨杆操作，功率目标设为 `INFINITY` 不限功率，保留 LQR 速度与位移反馈
- 超电实例、更新和收发由上板负责，接 Can(1)，`chassis_loop()` 每周期设 OFF 并更新，100 Hz 持续发送关闭指令，未接裁判时沿用默认 120 W 上限和 60 J 缓冲能量。OFF 关闭 DCDC，底盘仍按裁判功率与缓冲能量折算目标
- 上板将 `power_chassis_ref` 随控制帧下发：8 字节小端位域，bit0/1/2 为使能/自救/跳跃，bit3..15 前向速度（有符号 13 位，1e-3 m/s，±4.096 m/s），bit16..30 手动角速度（有符号 15 位，1e-3 rad/s，±16.384 rad/s，约 156 rpm），bit31..43 yaw（有符号 13 位，1e-3 rad），bit44..52 腿长，bit53..63 功率（11 位，0.25 W/LSB，四舍五入，有限值最高 511.5 W，2047 表示不限功率）。下板状态帧 bit0 为倒地，bit1..16 为实测航向角速度（int16，1e-3 rad/s），bit17..23 保留。下板移除 SuperCap，只调用 `set_power_ref()`
- 下板 Can(1) 每拍 3 帧：偶拍左腿两关节与双轮 0x200，奇拍右腿两关节加 0x003

## 轮腿功率控制

- `ChassisLeg` 只限制轮电机：`calc_lqr()` 把 LQR 轮力矩拆为驱动部分（位移、速度、航向角、航向角速度项）与平衡部分（腿倾角、俯仰项），交给 `limit_power()` 求 gain 并写入 T = 平衡 + gain·驱动
- 预测功率 Σ(T·ω + 0.75·R·(T/kt)²) ≤ power_ref，用 `MotorPower::solve_gain()` 求最大 gain；平衡部分不衰减，gain 为 0 时仍超出由缓冲能量承担；关节 DM8009 功耗不计入
- `wheel_resistance` 取 M3508 线间电阻 0.194 Ω，kt 取轮电机 `max_torque / max_current`；`power.estimate`、`power.gain` 供 SWD 与超电实测对照
