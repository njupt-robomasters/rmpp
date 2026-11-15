# 应用组织约定

仅在修改 `app/**` 时读取本文档

## 职责边界

- app 选择硬件和车型策略
- lib 接收句柄或引用
- 板卡 BSP 实现外设
- 校准组合键、断联保持使能等车型策略在 app 展开
- yaw 校零与偏移持久化在 app 直接操作电机和 KvStore：infantry 使用 yaw，sentry 使用 yaw1；启动将 `yaw_offset` 读入 `motor.config.offset`，校准时按当前输出轴角度、方向与减速比更新 offset，随后保存；两处 yaw 减速比均为 1
- lib 不为车型策略保留开关或流程
- 发射机构、底盘、超级电容、遥控器、裁判通信与 MiniPC 通信按值保存只读 `Config`，直接读取配置字段；PID 状态、运动学矩阵、校准值与其他运行状态独立保存，超级电容 RLS 的固定参数读取 `config.rls`
- 云台按值保存只读 `Config`，俯仰软件限位始终启用，各实例通过 `pitch_min`、`pitch_max` 配置编码器角度范围
- `GimbalClassic` 在 ECD 模式同步 yaw 的 IMU 目标，pitch 的 IMU 目标统一在编码器目标限位后回写
- 云台和 17/42 mm 发射机构通过 `Motor::set_output()` 设置原生输出，PID 单位和限幅由 app 按电机配置，省去外部 `track()` 回算；PID 自身的输出限幅与积分回算保留
- 双 yaw 共用速度 PID 输出，各电机按自身原生单位接收同一数值；sentry 的 DM4310 为 N·m、GM6020 为 A，参数待实车整定
- 17 mm 拨弹在 `update()` 内使用 STOP/FEED/REVERSE 状态机，失能、热量保护、零弹频或拨弹离线优先停止；拨弹独立于摩擦轮开关和在线状态
- 卡弹检测与退弹分别用 `block_elapsed`、`reverse_elapsed` 累加限幅后的 `dt`；连续命令电流饱和达到 `block_time` 后反向运行 `reverse_time`，方向和速度跟随当前有符号弹频；STOP 清累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID
- 17 mm 拨弹先由状态机确定目标速度，再统一计算速度 PID；FEED 使用上一周期命令电流检测卡弹，状态切换在下一周期改变目标速度

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
- 被多处使用或与机构业务无关的设备（姿态 `Attitude`、超电 `SuperCap`）由 app 调用 `update()`，结果再交给机构：每周期都要刷新的测量经机构 `update(...)` 参数传入（如底盘 `update(q, gyro)`、云台 `update(yaw, pitch)`），其余输入用 `set_*`（如 `set_power_ref()`、`set_gimbal_yaw()`、`set_chassis_wr()`）
- app 先更新共享设备再更新机构：如 `super_cap.update()` → `chassis.set_power_ref(super_cap.power_chassis_ref)` → `chassis.update()`
- 发送（`send()`、`write_can_frame()` 组帧）始终由 app 安排
- 模块间不互相引用状态
- 多模块共用判断放在 `app.hpp`

## jelly_sentry 摩擦轮

- 两个 P630 由 app 定义为 `Vesc`，绑定 CAN3，节点 ID 沿用 `0x21/0x22`，共用现有 VESC 扩展帧收发实现
- `Shooter17mm::update()` 通过 `set_speed()` 设置输出轴目标角速度（rad/s），由 Vesc 换算为 ERPM，使用电调内部速度环
- 整车、`jelly_sentry_debug/shooter` 与 `shooter_rub` 的摩擦轮 `send()` 暂时注释，继续接收反馈；其他电机保持原发送行为
- 当前显式保存迁移前 Vesc 参数：20 A、20 Nm、reduction 为 7（包含极对数），节点 ID、实际极对数及两侧安装方向需在启用发送前核对
- 2026-11-12：整车 app 及三个发射调试入口的相关对象文件经 Arm GCC 局部编译通过，无新增编译警告；未完整链接、烧录或硬件验证。`git diff --check` 通过，本机未找到 clang-format，未完成工具格式检查

## jelly_sentry EL05

- 拨弹 EL05 绑定 CAN2，MIT 标准帧控制ID 0x12、反馈ID 0x22，反馈Byte0必须为0x12；与pitch1的0x10/0x20、pitch2的0x11/0x21无ID重复
- 电机端参数由用户写入并断电重启：CAN_ID=18、CAN_MASTER=34、protocol_1=2、CAN_TIMEOUT=20000（约1秒）；主控不在线改参
- 整车和发射调试入口共用shoot实例，控制计算保持1ms，EL05按shoot_send_period_ms发送（默认2ms、500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答

## jelly_sentry 上游接口适配

- `GimbalFold` 仅构造绑定四电机与 Config；app 每周期更新 attitude，并通过 `gimbal.update(attitude.yaw, attitude.pitch)` 传入姿态，yaw1 零点与 KvStore 存取移到 app
- 大 yaw、pitch1、pitch2 经电机原生 `set_output()` 接收 Nm，小 yaw 7025 接收 A；折叠云台保留电机限幅后的 PID 跟踪及重力补偿，独立于上游普通云台的简化策略
- 单轴与拨弹测试显式限幅 dt 并传给 PID；底盘由 app 更新超电后传入 `power_chassis_ref`
- 受击方向显示与实测弹速继续读取裁判 `hurt_data`、`hurt_sequence`、`shoot_data`，这些已有字段为公开只读输出

## jelly_sentry CAN2 发送调度

- 整车控制计算保持1kHz；pitch1、pitch2和yaw2的发送周期可写变量放在gimbal.hpp各电机实例旁，EL05的shoot_send_period_ms放在shooter.hpp的shoot实例旁
- 默认周期：pitch1_send_period_ms=4（250Hz）、pitch2_send_period_ms=1（1000Hz）、yaw2_send_period_ms=2（500Hz，A1/94各250Hz）、shoot_send_period_ms=2（500Hz）
- 当前整车调试基线将 pitch1、pitch2、EL05 与摩擦轮发送注释关闭，仅底盘、大 yaw、小 yaw 保持发送；上述周期在对应发送代码启用时生效，合并时保留该开关状态
- 各变量为inline volatile uint32_t，运行时通过调试器写入，下一个发送判断使用新值，不清零已有计数。计数器保留在cpp，不作为调频入口；周期0等效每主循环发送，超过100ms可能触发主控反馈离线，7025建议周期至少2ms
- 失能、使能和清错帧替换当次发送，不额外增加发送频率。正常每请求一帧应答一帧时，主控发送2250帧/秒、反馈预计2250帧/秒，总计4500帧/秒；1Mbps标准8字节帧理论占用约50%~61%，不含错误重传与额外主动上报
- 频率是按1ms主循环计算的目标值，真实周期、反馈频率与7025断联是否改善仍需上板测量
- gimbal单轴调试共用pitch2周期；gimbal_motor通过send_period_ms引用当前测试电机的周期变量；shooter与shooter_shoot共用shoot周期。变量仅影响CAN发送，反馈按电机应答而定，不改变PID计算周期
