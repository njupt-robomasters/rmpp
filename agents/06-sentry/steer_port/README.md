# 老哨兵操控还原

状态：已改完并编译通过，待用户审计与上车

## 目标

`Joushua233/rmpp` 的 `old` 分支是确定能用的老哨兵代码。`app/steer_sentry` 已在 11-25 的 app_unify 中迁到当前 lib，本轮把仍缺失的操控行为按老哨兵还原，lib 未改动。

## 与 old 分支的差异与处理

| 行为 | `old` 分支 | 处理 |
| --- | --- | --- |
| 自主底盘输入 | 摇杆、键盘、导航逐轴相加，拨轮旋转也叠加 | 还原：平移与 VT13 逐轴相加后按 `vxy_max` 限幅，旋转与导航小陀螺相加后按 `wr_max` 限幅 |
| 过洞随头时的旋转 | 无过洞概念，`wr.input` 非零时优先 | 过洞分支直接取 `vt13.wr`，未操作时为 0，随头对准枪口 |
| 导航断联 | 速度项清零，摇杆仍生效 | 还原：去掉 `nav.is_connect` 停车，导航量清零后摇杆照常；大小 yaw 反馈丢失仍停车 |
| 导航上行 0xA4 | 上报目标点，按血量在中心/家之间切 | 还原：比赛中 HP≤120 回家、HP≥400 去中心，测试用测试点；`nav.send()` 200 Hz |
| 自主云台 | photo/右键的人工自瞄优先于自主状态机 | 还原：`vt13.is_aim && is_detect` 时直接跟随视觉角度，否则走锁定/扫描 |
| insta360 检查 | SCAN/INSTA360_CHECK/LOCK 三态 | 保持删除，新协议没有对应帧 |
| fn 测试摩擦轮 | 比赛才开，测试需 S 挡 | 保持常开 |
| N 挡发射 | 使能 | 保持失能 |
| 自检 yaw 旋转 | 所有模式 | 保持现状，自检与比赛互斥，实际等价 |

## 小范围行为变化

- 小陀螺插值改用导航与摇杆合成后的速度。老哨兵只按导航速度插值，现在与 jelly_sentry 的约定一致
- `is_go_home` 在退出自主时复位为去中心，与老哨兵 `handlePause` 一致
- 过洞分支允许操作手用拨轮旋转。老哨兵没有过洞标志，这条是新增组合

## 改动

- 新增 `app/steer_sentry/minipc.hpp`、`minipc.cpp`：`Aim`、`Nav` 从 `app.hpp` 移入，`minipc_loop()` 每周期填姿态、裁判数据和 yaw 编码器角，结构对齐 jelly_sentry 与 leg_up
- `app/steer_sentry/chassis.cpp`：加导航目标点状态机与 VT13 叠加
- `app/steer_sentry/gimbal.cpp`：自主分支加人工自瞄优先
- `app/steer_sentry/app.cpp`：CDC 错拍发送自瞄与导航，各 200 Hz
- `docs/minipc_protocol.md`：导航断联说明改为停止自动平移、摇杆仍有效

## 验证

- `cmake --build build/steer_sentry`：链接通过，RAM 22280 B（17.00%），FLASH 148932 B（14.20%）
- `agents/03-control/quad_steer/verify_offline.py`：底盘 9696 项检查与 6 个 ARM 对象编译通过
- `tools/format_code.py --check app/steer_sentry`：12 个文件通过
- 未烧录，未上车

## 上车核对

- 导航目标点 `(2.2, 4.7)` 与 `(0, 0)` 是否与新版上位机自定位同系
- 自主时摇杆和拨轮叠加后的方向与限幅
- 导航断联后摇杆驾驶是否符合预期
- 人工自瞄在自主期间接管与释放的过渡
