# 应用组织约定

仅在修改 `app/**` 时读取本文档

## 职责边界

- app 选择硬件和车型策略
- lib 接收句柄或引用
- 板卡 BSP 实现外设
- 校准组合键、断联保持使能等车型策略在 app 展开
- lib 不为车型策略保留开关或流程

## 启动与循环

- CubeMX 初始化后依次执行 `bsp_init()`、`app_main()`
- `app_main()` 依次轮询底盘、云台、发射
- 使用普通 loop，不引入操作系统或整车包装类

## 文件组织

- 公共内容放在 `app.hpp`
- 各模块在同名 `.hpp` 声明、`.cpp` 实现
- 文件名不加 `_loop` 后缀
- 配置和实例跟随所属模块
- 组件可独立使用
- 整合管理器引用外部实例

## 遥控器映射

- 不设置遥控器汇总层
- 底盘、云台、发射分别在对应 `.cpp` 直接读取 VT13、FSi6X
- 操控参数和跨周期状态使用文件内 `static`
- 模块间不互相引用状态
- 多模块共用判断放在 `app.hpp`

## jelly_sentry 摩擦轮

- 两个 P630 由 app 定义为 `Vesc`，绑定 CAN3，节点 ID 沿用 `0x21/0x22`，共用现有 VESC 扩展帧收发实现
- `Shooter17mm::update()` 通过 `set_speed()` 设置输出轴目标角速度（rad/s），由 Vesc 换算为 ERPM，使用电调内部速度环
- 整车、`jelly_sentry_debug/shooter` 与 `shooter_rub` 的摩擦轮 `send()` 暂时注释，继续接收反馈；其他电机保持原发送行为
- 当前沿用 Vesc 默认驱动参数（reduction 为 7，包含极对数），节点 ID、实际极对数及两侧安装方向需在启用发送前核对
- 2026-11-12：整车 app 及三个发射调试入口的相关对象文件经 Arm GCC 局部编译通过，无新增编译警告；未完整链接、烧录或硬件验证。`git diff --check` 通过，本机未找到 clang-format，未完成工具格式检查

## jelly_sentry EL05

- 拨弹 EL05 绑定 CAN2，MIT 标准帧控制ID 0x12、反馈ID 0x22，反馈Byte0必须为0x12；与pitch1的0x10/0x20、pitch2的0x11/0x21无ID重复
- 电机端参数由用户写入并断电重启：CAN_ID=18、CAN_MASTER=34、protocol_1=2、CAN_TIMEOUT=20000（约1秒）；主控不在线改参
- 整车和发射调试入口共用shoot实例，控制计算保持1ms，EL05按shoot_send_period_ms发送（默认2ms、500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答

## jelly_sentry CAN2 发送调度

- 整车控制计算保持1kHz；pitch1、pitch2和yaw2的发送周期可写变量放在gimbal.hpp各电机实例旁，EL05的shoot_send_period_ms放在shooter.hpp的shoot实例旁
- 默认周期：pitch1_send_period_ms=4（250Hz）、pitch2_send_period_ms=1（1000Hz）、yaw2_send_period_ms=2（500Hz，A1/94各250Hz）、shoot_send_period_ms=2（500Hz）
- 各变量为inline volatile uint32_t，运行时通过调试器写入，下一个发送判断使用新值，不清零已有计数。计数器保留在cpp，不作为调频入口；周期0等效每主循环发送，超过100ms可能触发主控反馈离线，7025建议周期至少2ms
- 失能、使能和清错帧替换当次发送，不额外增加发送频率。正常每请求一帧应答一帧时，主控发送2250帧/秒、反馈预计2250帧/秒，总计4500帧/秒；1Mbps标准8字节帧理论占用约50%~61%，不含错误重传与额外主动上报
- 频率是按1ms主循环计算的目标值，真实周期、反馈频率与7025断联是否改善仍需上板测量
- gimbal单轴调试共用pitch2周期；gimbal_motor通过send_period_ms引用当前测试电机的周期变量；shooter与shooter_shoot共用shoot周期。变量仅影响CAN发送，反馈按电机应答而定，不改变PID计算周期
