# 06 哨兵

状态：jelly_sentry、steer_sentry 的结构已并入 master，steer_sentry 已按 old 分支还原操控，待用户审计与上车

## 任务记录

- `steer_port/`：老哨兵操控还原（2026-11-26），自主时 VT13 叠加、导航目标点状态机、人工自瞄优先、0xA4 上行
- `upstream/`：Joushua233 上游审阅与移植（2026-11-24 至 11-25），导航叠加、温控、24 字节协议与接口适配
- `jelly_merge/`：jelly_sentry 及依赖从 rmpp-czk 并入 master，VT13 与 Referee 拆分适配
- `app_unify/`：哨兵与 leg 统一 app 结构，`sentry` 改名 `steer_sentry`，自主/遥控控制拆分

## 关键行为

- `steer_sentry` 保留蜂鸣器启动提示音，应用不驱动板载 RGB 灯
- `jelly_sentry` 使用 mc02：CDC1 自瞄、CDC2 导航；`steer_sentry` 使用 C 板，Aim 与 Nav 共用 CDC1，错拍各 200 Hz
- 自主控制按导航过洞标志切换：1 随头（FOLLOW），0 分离（DETACH）；导航速度都从 L 系按负小 yaw 角转到 H 系，旋转取遥控拨轮，jelly_sentry 平移叠加遥控、steer_sentry 过洞 0 走小陀螺
- `jelly_sentry` 的自主行为集中在 `sentry.hpp`、`sentry.cpp`，`*_loop()` 只按 `sentry_active()` 二选一；底盘按血量锁存切换导航目标点并在过洞 0 叠加固定小陀螺，云台扫描并在允许时锁定，发射机构摩擦轮常开；裁判自检阶段 yaw 叠加 180°/s 旋转由 `sentry_self_check_yaw_speed()` 统一给出，遥控分支与自主扫描共用
- 自主时 VT13 平移与旋转仍叠加：steer_sentry 平移按 `vxy_max`、旋转按 `wr_max` 限幅，导航断联后摇杆照常；jelly_sentry 只叠加平移
- steer_sentry 按血量在中心点与家之间切换导航上行目标点；任一 yaw 反馈丢失停车
- 自主云台：steer_sentry 的 photo/右键人工自瞄优先于锁定状态机
- 超电、UI、裁判和自瞄细节分别见 `../07-power/`、`../05-comm/`、`../02-conventions/application.md`

## 待办

双 CDC、坐标方向、随头方向、pitch1 失能、UI 显示，详见 `../TODO.md`
