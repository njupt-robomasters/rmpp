# 17 app 结构统一与哨兵控制拆分

状态：已并入 master，待用户审计与上车

## 目标

- jelly_sentry、sentry 统一到 leg 风格；sentry 改名 steer_sentry
- `app.cpp` 分为 `setup()`、`loop()`、`send()`、`app_main()`；删除 `misc.*`，RGB 在 `loop()` 更新
- 裁判与 UI 放在 app 层 `referee.hpp/.cpp`；leg_up 接入 Referee、RefereeSender 与自定义 UI
- 删除 insta360 行为；除 dummy、leg_down（及 `examples/rc` 遥控示例）外删除 FSi6X
- 哨兵按 `sentry_active()` 在 `control_by_vt13()` 与 `control_by_auto()` 间二选一
- 删除 app 中不必要的头文件和变量，修复发现的潜在问题

## 改动

- 目录：`git mv app/sentry app/steer_sentry`，预设 `sentry` → `steer_sentry`；KvStore 标记保留 `"sentry"`，避免格式化已标定的 yaw 零位
- 结构：两台哨兵 `app.hpp` 只放共享设备与 `rc_fn()`、`sentry_in_game()`、`sentry_active()`、`sentry_enable()`；模块 `.cpp` 先包含自身头再按需包含其他头
- 发送：leg_up、leg_down、dummy、两台哨兵、examples/motor 均拆出 `send()`；leg_down 因底盘计算拍需与发送拍一致，`send_cnt` 改为文件作用域并在 `send()` 末尾递增
- lib：`Aim` 删除 `insta360` 与其计时器；`Referee` 删除无人使用的 `hurt_timer`（受击判断改用 `hurt_sequence`）；`GimbalDualYaw` 增加 `turn_yaw()`
- 删除：steer_sentry 未被引用的 `yaw1_pid`、Ozone 注释宏；UI 中重复填写的 rub3/rub4

## 行为变化

- 自主时不再叠加 VT13 操控量，也不再允许人工接管导航；VT13 模式下底盘固定分离模式（原 fn 打开跟随，现 fn 进入自主）
- steer_sentry：导航坐标转换由 `+yaw2.angle` 改为 `−gimbal.yaw2.measure`，与 jelly_sentry 及 `minipc.md` 推导一致；增加 flag 仲裁（flag 0 跟随模式，未配跟随 PID 只平移）、Nav/yaw 离线停止、VT13 C 挡优先失能、N 挡发射失能、受击方向显示
- jelly_sentry：删除 FSi6X 后超电只按裁判连接、VT13 挡位与比赛阶段选择模式
- 自主射击：摩擦轮常开，视觉开火许可时按最高弹频（原比赛外 fn 测试时摩擦轮需 VT13 S 挡）
- 自主云台：低血量时不锁定（与原状态机等效），删除全景相机检查分支
- leg_up：超电按裁判功率上限与缓冲能量折算；发射机构接入热量，`heat_protect = 80` 待整定；UI 已接入

## 修复

- steer_sentry CAN1 每拍发 0x200、0x1FF、0x2FF、大 yaw 与裁判共 5 帧，C 板 3 个邮箱满时丢弃，大 yaw 与裁判帧基本无法发出；改为每拍至多 3 帧轮流发送
- jelly_sentry yaw 校零用单圈 `angle`，与文档要求的 `angle_multi` 不一致，多圈后零位错误
- steer_sentry R 键掉头直接写 GimbalDualYaw 公开 ref，改为 `turn_yaw()`
- F 键按住时每拍 `ui.init()` 反复清空队列，改为按下沿
- dummy `gimbal.hpp` 的 `static Imu imu` 在每个包含它的翻译单元各构造一份，`inline attitude` 引用其中之一，违反 ODR；改为 `inline`
- examples/motor 的 PID dt 未限幅，改用 `update_timer` 并按 `DT_MIN`/`DT_MAX` 限幅

## 验证

- `agents/tmp/build/<预设>` 隔离配置，编译 `rmpp_app`、`rmpp_lib`：jelly_sentry、steer_sentry、leg-leg_up、leg-leg_down、dummy、examples-motor 全部通过，0 警告
- `tools/format_code.py --check`（CLion 2026.2.2 随附 clang-format）通过
- `agents/07-power/super_cap/verify_jelly_sentry.py` 按新分支重写，120 种组合通过；`13-referee_relay` 三个主机测试通过
- 未做完整固件链接、烧录与硬件验证

## 上车核对

- steer_sentry：导航坐标符号（小 yaw 左转 90° 时雷达系向前应对应云台系向右）；CAN1 轮流发送后轮、舵、大 yaw 在线状态与裁判 UI 刷新
- 两台哨兵：fn 测试自主时摩擦轮常开；N 挡发射失能；自检阶段 yaw 旋转
- leg_up：CAN1 加裁判发送后的总线负载与下板关节在线状态；UI 刷新率、受击方向；裁判功率上限下的底盘表现；热量保护阈值
