# jelly_sentry 并入 master

2026-11-20 用户授权，把 `../rmpp-czk` 的 jelly_sentry 和它依赖的库并入当前工作区。来源为 rmpp-czk HEAD `65b91de` 加其工作区未提交的 Nav 协议与 pitch1 失能改动（czk 记录为已验证、待审计），基线 master `b9d4fce`

## 范围

- 新增：`app/jelly_sentry/`、`lib/motor/el05.*`、`agents/docs/minipc.md`、jelly_sentry 预设（`tools/update_presets.py` 生成）
- 库：`lib/minipc/nav.*` 换成 17 字节导航协议，构造为 `Nav(Cdc, Config)`；`Shooter17mm` 卡弹检测改用 `torque_ref` 与 `config.block_torque`，默认 `block_time` 0.5 → 0.2 s；`Referee` 公开 `hurt_data`、`shoot_data`，新增公开 `hurt_sequence`，收到 0x0206 时递增
- sentry：Nav 迁移为 `Nav(Cdc(1), {})` 和 `nav.command`，删除只在本地赋值的 GO_CENTER/GO_HOME 占位流程，导航离线时不叠加导航速度
- 未并入：`app/jelly_sentry_debug/`（按用户要求删除）及仅供 imu_calib 使用的 `Imu::temperature` 公开；`boards/mc02` 的 CubeMX 6.18 重新生成差异（仅版本号和 USER CODE 标签名）
- agents：czk 的 docs、TODO 和任务记录一并迁移，任务目录 12、13 改号为 `14-upstream_merge_review/`、`15-minipc_nav/`，其 `agents/out/` 产物未复制；记录中的调试入口和验证结果属于 czk 历史，按原样保留

## 适配 master 接口

- VT13 改读操控量：`vt13.vx/vy/wr`、`yaw_speed/pitch_speed`、`bullet_speed/bullet_freq`、`is_enable`、`is_aim`，原始量从 `vt13.rc` 读取；满量程、键盘加减速度、弹速 5 m/s、最高弹频 20 发/秒写在 `misc.hpp` 实例配置中，与迁移前常量一致
- FSi6X 保留：摇杆乘 VT13 满量程后叠加；使能、fn、校准组合、超电模式和断联时射击的拨杆映射不变
- Referee 改为 `{.can = Can(1)}`，新增 `RefereeSender`；主循环在 `referee.update()` 后调用 `set_robot_id()`、`send()`，UI 经 `RobotUi::update(referee_sender, {...})` 填写，启动时及按 F 键时 `init()`
- 主循环改为局部 `static` 的 `app_timer`、`app_monitor`，删除 `cpu_usage`

## 行为变化

- 鼠标云台的角加速度限制删除，与 sentry 一致
- 云台 pitch 取 `-vt13.pitch_speed`，保持迁移前摇杆上推目标增大的方向
- VT13 在线时射击：拨盘取 `vt13.bullet_freq`，S 挡鼠标左键也能射击；按住 photo 或右键自瞄时需要视觉开火许可，右键双击强制开火（迁移前 VT13 在线时只有 trigger 控制拨盘，鼠标分支不可达）
- 底盘转速未知（NaN）时，云台小陀螺前馈和 UI 转速改为 0
- UI 的 pitch 状态位改用 `pitch_motor` 字段，弹频显示 `vt13.config.bullet_freq`

## 验证

- jelly_sentry、sentry 两个预设的 `rmpp_app`、`rmpp_lib` 对象用 CLion 自带 CMake/Ninja 与 arm-none-eabi-gcc 编译，零警告零错误；构建目录 `agents/tmp/build-<preset>`，未完整链接固件
- 修改的手写源码已用 CLion 自带 clang-format 格式化，`git diff --check` 通过
- 未烧录、复位或操作硬件

## 上车核对

- 云台 pitch 摇杆与鼠标方向
- S 挡鼠标左键射击与自瞄开火许可
- 受击方向圆弧在小陀螺时是否保持世界方向
- czk 记录的待核对项：EL05 protocol_1、雷达安装偏角、导航随头方向、pitch1 失能状态、超电各模式（见 TODO 对应条目）
