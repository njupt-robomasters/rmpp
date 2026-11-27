# 06 哨兵

状态：接入上游 `1cb6bbf` 后保留本地自瞄窗口；当前 pitch2 已按用户要求恢复编码器速度反馈及原 PID，保留用户调整的 −0.14 rad 下限，本次局部编译和格式检查通过，待用户审计

## 任务记录

- `gimbal_imu/`：当前折叠云台 yaw1/yaw2 世界系反馈及 50 Hz IMU 速度低通、±50° 自瞄窗口，以及 pitch2 编码器速度内环与 IMU 反馈试用撤回记录
- `yaw_gyro/`：历史小 yaw 关节轴 IMU 速度内环接入记录，已由当前世界 Z 反馈替代
- `steer_port/`：老哨兵操控还原（2026-11-26），自主时 VT13 叠加、导航目标点状态机、人工自瞄优先、0xA4 上行
- `upstream/`：Joushua233 上游审阅与移植（2026-11-24 至 11-25），导航叠加、温控、24 字节协议与接口适配
- `jelly_merge/`：jelly_sentry 及依赖从 rmpp-czk 并入 master，VT13 与 Referee 拆分适配
- `app_unify/`：哨兵与 leg 统一 app 结构，`sentry` 改名 `steer_sentry`，自主/遥控控制拆分
- `yaw_window/`：识别后小 yaw 优先跟踪、超限大 yaw 追随及历史回归
- `yaw_pid/`：大小 yaw 实测、历史阶跃和正弦入口；2026-11-28 已从 Ember 保存当前四组 PID 参数

## 关键行为

- `steer_sentry` 保留蜂鸣器启动提示音，应用不驱动板载 RGB 灯
- `jelly_sentry` 使用 mc02，CDC1 自瞄、CDC2 导航，错拍各250 Hz；`steer_sentry` 使用 C 板，Aim 与 Nav 共用 CDC1，错拍各200 Hz
- 两车均按雷达系接收导航速度，经负小 yaw 转到头部系后叠加遥控；jelly_sentry 固定 90 rpm 小陀螺、过洞标志暂不处理，steer_sentry 按过洞标志 1 随头、0 小陀螺
- 自主时 steer_sentry 叠加VT13平移与旋转并限幅，导航断联后摇杆照常；jelly_sentry 在线时叠加平移并保留相加结果
- 两车按血量在中心点与家之间切换导航上行目标点；jelly_sentry 在 `sentry.cpp` 用 `is_go_home` 以中心 `(6.0, -3.8)` m 为目标，不依赖裁判 RFID，`<100` HP 回家 `(0.15, -0.15)` m 直至满血并期间屏蔽自瞄与拨弹
- jelly_sentry 整车使能取 `vt13.is_enable`（N、S 挡），C 挡和断联时不使能；正赛中断联由 VT13 保持挡位，链路抖动不改变使能
- jelly_sentry 自主云台不在回家途中且识别到目标时启用小 yaw ±50° 优先窗口。yaw1/yaw2 四组 PID 参数已从 Ember 保存到源码，待用户审计；pitch2 保留编码器速度内环和原 PID，编码器软限位为 −8°～18°
- 摩擦轮正常遥控采用上游：默认 18 m/s，N 挡弹速为零但发射机构保持使能；自主发射比赛中自动开轮，场下测试模式需手动 S 挡，S 挡同时作为开火批准。回家期间禁拨弹
- 自主云台：steer_sentry 的 photo/右键人工自瞄优先于锁定状态机
- 超电、UI、裁判和自瞄细节分别见 `../07-power/`、`../05-comm/`、`../02-conventions/application.md`

## 待办

双 CDC、坐标方向、随头方向、pitch1 失能、UI 显示，详见 `../TODO.md`
