# 规划与待办

只保留未完成事项：待用户确认的决策、待上车核对、未解决的硬件问题和后续任务。每条写现状和记录位置；办结后删除，长期结论写进主题文档或任务记录。

## 待用户确认

### Referee 串口分层

- 现状：`Referee` 直接收 CAN（`lib/referee/referee.hpp` 的 `Config::can`），leg_down 用 `RefereeRelay` 把裁判串口字节转成 CAN 0x00/0x01，leg_up 已接入 `Referee`、`RefereeSender` 和 UI
- 旧方案（09766fa 记录，未实施）：`Referee` 只接 UART，由 app 实现 UART-over-CAN 桥接；影响 Uart 绑定入口、两板分派、三处实例和主机测试
- 待定：是否还需要改成 UART 抽象；不采用就删除本条

### pitch1 折叠/展开

- 当前 pitch1 固定失能并发送失能帧；后续启用时再定目标角、编码器零点和动作流程，见 `agents/05-comm/minipc.md` 与 `05-comm/minipc_protocol/README.md`
- 导航上行目标坐标、pitch1 折叠时序接入前先确认与上位机的坐标约定

### 轮腿控制目标

允许车体倾斜、优先定位的方案只分析未实施；LQI、`theta_ref = 0` 未确认，见 `04-leg/lqr/README.md`

## 待上车核对

### 轮腿 leg_up / leg_down

- 功率分配：`wheel_power` 与超电 `power_chassis` 对照；超目标时俯仰和腿角；低腿长缓加速先试；上板断联时协议清零、底盘失能，见 `04-leg/power_alloc/README.md`
- 航向前馈：方向、`cos(yaw)` 投影、小陀螺与跟随切换瞬间、限功率时平衡稳定性，见 `04-leg/chassis/README.md`
- 自瞄链路：CDC1 枚举、坐标系方向、人工接管与超时恢复、弹频放行，见 `05-comm/minipc_protocol/README.md`
- 裁判与 UI：`referee_relay` 丢字节计数、0x0301、UI 显示、超电模式，见 `05-comm/referee_relay/README.md` 与 `06-sentry/app_unify/README.md`
- 首版 LQR 参数对照：只换 K 后的抖动和定位，见 `04-leg/lqr/README.md`
- 力矩分配：额定力矩上限、姿态组优先与运动组退让，见 `04-leg/torque_alloc/README.md`

### 哨兵 jelly_sentry / steer_sentry

- 双 CDC 枚举与隔离、雷达与大 yaw 安装偏角、负小 yaw 坐标方向、随头方向、pitch1 失能状态，见 `05-comm/minipc_protocol/README.md`
- steer_sentry 老哨兵操控还原：导航目标点 `(2.2, 4.7)` 与 `(0, 0)` 是否与新版上位机自定位同系；自主时摇杆与拨轮叠加后的方向和限幅；导航断联后摇杆驾驶；人工自瞄接管与释放，见 `06-sentry/steer_port/README.md`
- 超电：0x051 电压电流符号与量程；CHARGE 稳态 `power_cap` 不为负；BOOST 在 10/12 V 的补偿衰减；断联重连；UI 按电容电压平方显示储能（满电默认 23.85 V），见 `07-power/super_cap/README.md`
- UI 椭圆/圆弧半轴按协议修正后的显示；无选手端时 UI 请求被丢弃，见 `05-comm/referee_relay/README.md`
- 机构参数按实测样机，温度、云台和发射机构保留本地实现，见 `06-sentry/upstream/README.md`

### 其他应用

- 四舵轮底盘上车，见 `03-control/quad_steer/README.md`
- dummy 靶车映射与 follow PID，见 `03-control/dummy/README.md`
- 拨弹状态机、GimbalClassic 限位与加速度规划，见 `03-control/shooter/README.md`
- VT13 键盘斜坡、云台与发射映射，见 `03-control/vt13/README.md`
- 轮腿上下板 UI 槽位与 6 个电机状态，见 `04-leg/link_ui/README.md`

## 未解决硬件问题

- EL05/CAN2：7025 + PCAN 独立总线部分 ID 使反馈间隔从 2 ms 跳到十几 ms；待核对真实请求间隔、分命令应答数、错误帧和 CANID 滤波器开关。EL05 已改 CAN2 私有扩展帧、电机与主机节点保留 0x12/0x22，电机侧 `protocol_1` 需核对（记录为 0），见 `01-board/mc02_bsp/README.md` 与 `agents/01-board/motor.md`
- 两个 DM4310 接入 EL05 后丢反馈，实机原因未确认；EmberProbe 仍从旧工作区 `uart2can-master` 查找桥接，需把会话工作目录切到 rmpp 后读总线错误状态
- mc02 OUT_5V 供电的是哪一路 CAN 接口：看原理图或实测后写进 `boards/mc02/bsp/include/board/power.hpp` 注释
- KvStore 升级 FlashDB 2.2.0 后，C 板已有 yaw_offset 可能需重新标定
- 蜂鸣器：mc02 `examples-melody` 待上板；C 板复音算力未实测，整车只用普通模式
- leg_up 发射机构 `heat_protect = 80` 待整定

## 后续任务

- VQF 姿态算法：原样引入官方 C++ 实现（确认单精度选项），封装 `Vqf` 与 `QuaternionEkf` 并列
- 导航上行目标点坐标：steer_sentry 已按老哨兵血量状态机发送中心/家，坐标沿用旧赛场数值；需与上位机确认同系，或改装 `Referee::map_command` 的选手端指令
- 虚拟 IMU：云台 IMU + 关节角合成底盘姿态，替代下板 IMU
- 转向指令限幅与高速段功率模型：轮速能力不足时 `wr` 长期饱和，模型按指令力矩计算会低估实际功率，见 `04-leg/spin_power/README.md` 遗留
