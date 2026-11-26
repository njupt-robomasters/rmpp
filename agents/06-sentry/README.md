# 06 哨兵

状态：jelly_sentry、steer_sentry 的结构已并入 master，待用户审计与上车

## 任务记录

- `upstream/`：Joushua233 上游审阅与移植（2026-11-24 至 11-25），导航叠加、温控、24 字节协议与接口适配
- `jelly_merge/`：jelly_sentry 及依赖从 rmpp-czk 并入 master，VT13 与 Referee 拆分适配
- `app_unify/`：哨兵与 leg 统一 app 结构，`sentry` 改名 `steer_sentry`，自主/遥控控制拆分

## 关键行为

- `jelly_sentry` 使用 mc02：CDC1 自瞄、CDC2 导航；`steer_sentry` 使用 C 板，Aim 与 Nav 共用 CDC1
- 自主控制按导航过洞标志切换：1 随头（FOLLOW），0 小陀螺（DETACH）；两种底盘分别按 H 系/L 系换算
- 导航掉线退出自动旋转，平移只取 VT13；任一 yaw 反馈丢失停车
- 超电、UI、裁判和自瞄细节分别见 `../07-power/`、`../05-comm/`、`../02-conventions/application.md`

## 待办

双 CDC、坐标方向、随头方向、pitch1 失能、UI 显示，详见 `../TODO.md`
