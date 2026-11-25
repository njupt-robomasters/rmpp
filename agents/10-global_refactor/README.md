# 全局代码优化与类布局重构

基线：`8f01151`（改动前已提交工作区）。长期约定见 `agents/docs/conventions.md` 与模块文档，历史修改涉及、`docs/architecture.md`、`docs/application.md`、`docs/motor.md`、`docs/bsp.md`

## 改动

- 缺陷：GM6020、M2006 转速与电流按无符号解析（`c9ac138` 丢失 `int16_t`），反转时读成大正数；GM6020、DmMotor 在 raw 中先除一次减速比，基类再除一次（现有型号减速比均为 1，未暴露）
- Timer 只计时，删除 min/max 与未使用的 `dt` 成员；c_board 与 mc02 的 `elapsed()` 行为原本不一致
- `Pid::calculate(dt, err, derr)`，删除内部计时器；构造或 `clear()` 后首帧不差分
- 持有方在 `update()` 开头计一次周期并按 `DT_MIN`/`DT_MAX` 限幅：ChassisMatrix、ChassisSteer、ChassisLeg、Gimbal、Shooter17mm/42mm、QuaternionEkf、Imu 加热、SuperCap（只限上限 0.1 s）
- Motor：Drive 删除 kt，为零字段由型号常量补齐，电流与力矩按 `max_torque / max_current` 换算；新增 `set_zero()`；Vesc 无型号默认值，摩擦轮由 app 填 M3508 常量
- GM6020 `BackEmf{gain, ke}` 改为构造参数 `float ke = 0`
- 底盘不再持有 SuperCap：app `super_cap.update()` 后 `chassis.set_power_ref(super_cap.power.chassis_ref)`
- 云台不再持有 Attitude：app `attitude.update()` 后 `gimbal.update(attitude.yaw, attitude.pitch)`
- 跟随 PID 在不跟随时 clear；ChassisLeg 腿角 PID 在非自救时 clear
- 历史类布局已由下方“外部 worktree 类布局重构”替换
- app 函数内 `static` 提到文件作用域；sentry 计时器改名 `<用途>_timer`；Aim 计时器改名

## 行为变化

- dummy
  - 跟随 PID 在分离模式、摇杆旋转、失能时 clear；切入跟随的首帧不再有旧误差差分冲击（原首帧 dt 被限幅为 0.01 s）
  - 云台 yaw PID 与目标积分改用同一个限幅后的 dt，1 kHz 正常运行时与原来相同
  - 底盘 X630、云台 RMDL7025 的 drive 取型号默认值，与原显式值相同
- leg_down
  - 腿角 PID 在非自救时 clear，自救首帧不差分；其余 PID 给定 derr、ki 为 0，dt 不影响输出
  - 电机 drive 取值与原来相同（DM8009 kt = 40/40 = 1；轮 kt = max_torque/max_current = WHEEL_KT）
- leg_up
  - GM6020 转速符号修复：反转时 speed 原来是错误的大正数，速度环行为改变（变为正确）
  - 云台目标积分 dt 增加 0.01 s 上限
- sentry：原本因缺 Vesc 常量编不过；摩擦轮改填 M3508 常量；轮 M3508、DM4310 的 max_current 原为 0（set_current 恒为 0），现取型号值
- DM4310、DM6006、RMDL7025 的 MAX_TORQUE/MAX_CURRENT 与原 KT 不一致，按用户决定以满量程比为准；只影响电流或力矩的换算显示与非主输出路径

## 上车核对

- leg_up：yaw、pitch 反转时 speed 为负且量级正常；云台速度环稳定
- dummy：切换跟随时底盘无突跳；超电模式切换与原来一致
- leg_down：自救流程与原来一致

## 复现

```bash
python tools/check_builds.py --cmake <CLion cmake> --ninja <CLion ninja>
```

`agents/tmp/10-global_refactor/` 下为一次性编辑脚本，不入库

## 外部 worktree 类布局重构

- 基线：`795e263`；分支：`refactor/class-layout`；worktree：`../rmpp-class-layout`
- 范围：项目手写 class、对应实现与调用点，CubeMX 生成代码及第三方原始代码保持原样；infantry 仅迁移共同接口调用，跳过编译验证
- 常量集中在 class 最前面，public 优先；全部变量在函数前，输出与诊断按观察顺序排列，配置和设备居中，跨周期状态及 Timer 收尾
- 变量名去尾下划线，权限按当前调用收紧；子类确需的变量用 protected，仅供 SWD 的内部量用 private
- 函数按 public / 必要的 protected / private 排列，private 回调 can_callback 在前；.cpp 和 .tpp 定义跟随声明，构造初始化列表跟随变量顺序
- Config 仅作为构造输入，实例配置展开并按相似含义分组；Pid 保留指向调用方可调配置的只读指针
- 电机 bus/drive/install/raw 展平，实例使用 max_current/max_torque/offset、protected 通信配置与 raw_* 标量；固定字段加 const
- SuperCap 的 voltage/current/power/energy 展平，RLS 配置与窗口状态保留在 rls，功率管理算法保持原有流程
- 单一设备绑定的 Motors 结构与 motor 实例就地声明；局部变量也复用的请求、配置等类型保留名称
- 根 AGENTS.md 精简为入口，详细编码规则迁入 agents/docs/conventions.md，协作、验证与维护规则迁入 workflow.md，README 提供按需读取索引

### 接口变化

| 原路径 | 当前路径 |
| --- | --- |
| 构造 `.bus = {.can = …, .rx_id = …}` | 构造 `.can = …, .rx_id = …` |
| 构造 `.drive = {.max_current = …}` | 构造 `.max_current = …` |
| 构造 `.install = {.is_invert = …, .offset = …}` | 构造 `.is_invert = …, .offset = …` |
| `motor.config.drive.max_current/max_torque` | `motor.max_current/max_torque` |
| `motor.config.install.offset` | `motor.offset` |
| `super_cap.power.chassis_ref` | `super_cap.power_chassis_ref` |
| `super_cap.power.battery` | `super_cap.power_battery` |
| `imu.config_.gyro_offset/gravity`（SWD） | `imu.gyro_offset/gravity`（SWD） |

删除的嵌套层原用于参数与反馈分组，现由同组标量、空行和注释表达；Config 输入仍保留指定成员初始化。最小权限调整会限制外部直接访问诊断字段，SWD 可观察这些 private 字段。

### 验证与复现

最终检查：53 个 class 的变量在前、无尾下划线、常量在顶部、private can_callback 在前，以及声明/实现排序检查通过。101 个受影响工程手写源码格式检查通过，额外检查模板和主机回归源码；git diff --check 通过。14 个预设对象编译通过，两板公共库与手写 BSP 编译通过，四组主机差分回归输出逐字节一致。

本轮验证对象库与主机行为，未链接完整固件，未烧录或执行硬件操作。

```sh
cmake --preset dummy
cmake --build build/dummy --target rmpp_lib rmpp_bsp rmpp_app --parallel 8
cmake --preset examples-cdc
cmake --build build/examples-cdc --target rmpp_lib rmpp_bsp rmpp_app --parallel 8
python agents/10-global_refactor/layout_regression.py --baseline 795e263
```

- 应用对象编译：dummy、examples-attitude/cdc/gimbal/kv_store/melody/motor/rc/referee/rgb、leg_down、leg_up、sentry、test，共 14 个预设
- 公共库与手写 BSP：dummy 覆盖 C 板，examples-cdc 覆盖 mc02
- 主机差分回归：layout_motor_regression.cpp 验证八类电机反馈、限幅、零点、离线处理、CAN 命令及 17 mm 发射机构电流/力矩输出；sim.cpp 验证超电容量/ESR；power_sim.cpp 验证功率与虚拟缓冲能量；layout_melody_regression.cpp 比较直驱音符时序、带伴奏和纯旋律的复音浮点采样。分别编译基线与当前实现，要求输出逐字节一致
- `layout_regression.py` 只读取指定 Git 基线，提取文件与构建产物放在`agents/tmp/10-global_refactor/layout_regression/`；包含生成和第三方目录的工程文件未改动
- 保留的应用告警：motor 示例两个未使用函数；leg_up/gimbal.cpp 的 is_aim 未使用
- 构建日志和差分结果位于 agents/tmp/10-global_refactor/；最终状态以本节对应执行结果为准，早期失败尝试也保留在 agents/tmp/10-global_refactor/ 便于追溯

类布局重构已并入 master，外部 worktree 随合并移除；待用户审阅 SWD 展开顺序与接口后决定上车测试。

### 固定配置收拢

用户确认撤销固定配置逐项展开的写法：云台、17/42 mm 发射机构、矩阵/舵轮/轮腿底盘、超级电容、QuaternionEkf、Melody、FSi6X、VT13、Referee、Aim、Nav 按值保存 `const Config config`，使用处直接读取字段。超级电容 RLS 固定参数统一读取 `config.rls`；PID 状态、运动学矩阵及蜂鸣器预计算的采样周期、增益步长和采样数继续独立保存。电机解析后的量程/零点与 IMU 校准状态按原职责维护。

验证：本轮新增修改的 13 个公共库源码对象编译通过，dummy、leg_up、leg_down 应用对象编译通过；26 个新增修改源码的格式检查与 diff 检查通过。复用本目录现有四组主机回归，对比修改前 HEAD 与当前代码，电机及 17 mm 发射机构、超电辨识、功率管理、蜂鸣器的输出逐字节一致，产物位于 `agents/tmp/10-global_refactor/config_regression/`。逐文件核对确认函数体仅改变配置字段访问路径。未执行完整固件链接或硬件操作。

轮腿补充调整：力映射角度按实际顶点命名为 ∠OCB（cos_ocb、tan_ocb）；设置接口统一为 set_allow_rescue(allow_rescue)、set_trigger_jump(trigger_jump)、set_leg_length(leg_length)，同步 leg_down 调用点；update_kin 合入 update，保留原执行顺序。轮腿源码与 leg_down 的 chassis.cpp、app.cpp 对象编译通过，格式及 diff 检查通过。

主线合并：重构提交为 7aee5ea、903f01f。主线原有 13 个暂存文件保留暂存；冲突按双方意图合并，保留首版 Python LQR 系数、轮端力矩与减速比、位移误差乘 10 后限幅、位移积分限幅及跳跃时序，新增 x_err_max 对应只读成员和构造初始化。合并后在主线重新编译轮腿及 leg_down 的 chassis.cpp、app.cpp 对象通过。外部 worktree 的忽略构建产物与临时日志随目录移除，复现源码已入库。

### Timer 调试与电机配置收拢

- Timer 两板均在 `restart()` 更新 private `dt` 并返回，保存未经限幅的秒数，初值为 0；`elapsed()` 保持只读
- Motor 按值保存 `Config config`，通信、超时、量程、减速比、安装方向与零点直接使用 config；量程和减速比的零值仍在构造时补齐
- 删除 `set_offset()`、`set_zero()`，app 直接维护 `config.offset`；infantry/sentry 启动读取、在线校准及持久化已迁移，校零按当前输出轴角度、方向和减速比换算
- 删除原来供 `set_zero()` 读取的 `position_multi` 成员，多圈位置换算直接在反馈处理中完成；功率计算和发射机构读取量程的调用点同步迁移
- 新增 `motor_config_test.cpp`，覆盖默认值补齐、显式配置、电流/力矩限幅与换算、反向反馈、外部校零和超时；历史布局测试保留当时接口，本轮使用新测试及任务 11 的原生输出/拨弹测试

验证：两板 timer.cpp ARM 对象编译通过；配置回归及任务 11 主机测试通过；24 个相关源码 ARM 对象编译通过，无警告。21 个工程源码及 DM 模板、新测试的格式检查与 `git diff --check` 通过。未执行完整固件链接或硬件操作；新增对象和测试产物位于 `agents/tmp/10-global_refactor/`，任务 11 验证产物沿用 `agents/tmp/11-shooter_state/`。

### angle、多圈零点与配置优先布局

基线 `da3f164`，外部 worktree `../rmpp-motor-angle`，分支 `refactor/motor-angle-layout`。主工作区的 TODO 修改随主线提交保存；2026-11-17 已合并至 master，外部 worktree 已移除，分支保留。

- 电机及调用点统一使用 `angle`、`raw_angle`；VESC 采用 `set_angle()`、`pid_angle`，双 yaw 采用 `*_angle_pid*`；底盘平移及导航坐标继续使用 position
- 新增输出轴连续角度 `angle_multi`，计算为 `dir * (raw_angle_multi - config.offset) / config.reduction`，与单圈 angle 共用电机侧 offset；运行时修改 offset 后在下一次反馈刷新，原始累计不受校准影响
- 首个有效角度帧直接保存原始角度，避免首帧大于 π 时丢失一圈；后续保持最短角差累计，离线保留累计。RMD 电流应答与 VESC 状态应答传入 `can_callback(false)`，避免无角度帧提前初始化计数
- infantry/sentry 外部校零使用 angle_multi，同时清零单圈与连续角度，继续由 app 持久化 offset
- Pid 配置指针及其他类配置成员移到实例变量最前；Timer 保持在变量区末尾；update 放在所属类最后，其他方法及对应定义提前，构造初始化列表同步排列。规则更新至 AGENTS.md 与 conventions.md

验证：两板公共库、两板 IMU BSP、dummy/infantry/sentry/leg_down/leg_up/motor 示例/test 与 mc02 CDC 示例共 82 个真实 ARM 对象编译通过，启用 `-Werror=reorder`，无告警；复用数据库时跳过已删除的 examples/gimbal 源码。电机配置回归覆盖首帧角度大于 π、无角度应答先到、正反转跨圈、方向、三种减速比、连续校零、重复帧、offset 修改及离线恢复。任务 11 原生输出/拨弹回归与 GimbalClassic IMU 的 3280 项检查通过。此次仅编译对象和执行主机测试，未链接固件或操作硬件。

复现 ARM 对象编译（source-root 提供已有编译数据库，mc02-build 为当前 worktree 的 mc02 配置目录）：

```sh
python agents/10-global_refactor/verify_angle_layout.py --source-root <原工作区> --mc02-build <mc02构建目录>
```

对象及逐文件日志位于 `agents/tmp/10-global_refactor/angle_layout/arm/`，电机回归源码沿用 `motor_config_test.cpp`；云台与拨弹回归入口沿用任务 11 的 `verify_gimbal_imu.py` 和 `verify.py`。

合并验证（2026-11-17）：主线快照 `c576037`、外部实现 `d888b40`、外部记录 `c48610b`，合并提交 `87ef1f8`，无冲突。合并后 82 个角度/布局 ARM 对象及两板 Timer 对象编译通过；电机配置与多圈角度、原生输出/拨弹、云台 IMU 3280 项检查通过。57 个相关工程源码格式检查与 diff 检查通过。外部日志归档至 `agents/tmp/10-global_refactor/angle_layout/external_logs/`。未执行完整固件链接、硬件操作或推送。

### 电机可选反馈 Feedback

- `Motor::can_callback(const Feedback&)` 取代 `can_callback(bool is_angle_frame)`；protected `Feedback` 含 `std::optional<float>` current、torque、speed、angle（电机侧原始量），子类用指定成员初始化写出本帧携带的量
- 基类每次回调刷新在线状态；有值的量修正方向与减速比，留空的量保持上次反馈；电流、力矩只传其一时按量程换算另一项；只有 angle 有值时推进多圈累计，角度输出每次按当前 offset 重算
- 删除 protected `raw_current/raw_torque/raw_speed` 及 `raw_angle_last`；`raw_angle` 改为 private，兼作上次角度参与最短角差累计
- 子类不再换算电流/力矩：DM 传 torque，DJI/X630/RM6623/RMD/VESC 传 current；RMD 0xA1 传 current/speed、0x94 只传 angle；VESC STATUS 传 current/speed，其余状态帧传 `{}`
- 行为差异：RMD 断联清零后，若先收到 0x94 角度帧，电流/力矩保持 0，直到 0xA1 应答到达（原实现会用旧 raw_current 恢复旧值）
- `Feedback` 成员带 `= {}` 默认初始化，避免主机 `-Wextra` 对省略的指定成员报 missing-field-initializers

验证：`verify_motor_feedback.py` 复用 `build/` 下 8 个 C 板预设的编译数据库，编译全部 lib/、app/ 手写源码共 253 个 ARM 对象（含 DM 模板实例化），0 失败 0 警告；产物在 `agents/tmp/10-global_refactor/motor_feedback/arm/`。`motor_config_test.cpp` 新增力矩单传换算、空反馈保持上次值的检查，与任务 11 原生输出/拨弹、GimbalClassic IMU 3280 项回归均通过。未链接固件，未操作硬件。

```sh
python agents/10-global_refactor/verify_motor_feedback.py
```

## 2026-11-17 成员六组排序、函数排序与最小权限

基线 `ba752f6`；外部 worktree `../rmpp-class-order`，分支 `refactor/class-member-order`。主工作区已有 `app/leg_up/shooter.cpp`、`lib/robot/gimbal/gimbal_classic.{hpp,cpp}`、`lib/robot/shooter/shooter_17mm.hpp` 的未提交修改，本轮从提交基线工作，保留主工作区内容。

- 类成员依次为构造参数、set 输入、SWD 观察状态、PID 组件、跨周期历史状态、Timer；构造参数按接口顺序，设备绑定靠前；Timer 按其他 timer、callback_timer、update_timer 排列
- 函数依次为构造函数（含 protected 构造）、set_enable、其他 set 函数、其他公开接口、update、内部辅助函数；内部辅助函数按入口首次调用顺序深度优先排列，调用者在前、被调用者紧随；.cpp 与 .tpp 定义跟随声明顺序
- 保留成对的 ref/measure 结构；状态机当前阶段位于 PID 前，历史标记与累计时间位于 PID 后；构造初始化列表同步声明顺序，保留原有计算与默认值
- 类型与实例权限相同时保持结构体后直接声明实例；公开配置类型与私有配置副本分开。云台、17 mm 发射机构、裁判的配置副本、17 mm 设备绑定、42 mm 观察量、Imu 温度、Melody 监测量和无外部引用的型号常量收紧为 private；底盘 vx/vy 仅供派生类使用，收紧为 protected
- 权限核对覆盖 app、公共库及现有测试调用；超电仿真读取的辨识/功率输出、双 yaw 差分回归读取的 yaw1/yaw2 保留 public；SWD 观察本身无需公开变量
- 同步 AGENTS.md、conventions.md 与相关模块文档；主工作区、生成代码和第三方代码未改动

验证结果：

- 两板公共库、两板 IMU BSP、dummy/infantry/sentry/leg_down/leg_up/motor 示例/test 与 mc02 CDC 示例共 82 个真实 ARM 对象通过，开启 `-Werror=reorder`；最终两处权限整理涉及的四个对象补编通过
- 原生电机输出与拨弹状态机回归通过；GimbalClassic IMU 3280 项检查通过
- 复用现有测试，对比基线与当前电机可选反馈、超电容量辨识、超电功率仿真、蜂鸣器播放，四组输出逐字节一致
- 30 个修改的手写代码文件格式检查与 git diff --check 通过，86 份 ARM 编译日志均为空；结束检查时主工作区另出现并行的根 TODO.md 修改，保持原样
- 本轮仅编译对象与执行主机回归，未链接完整固件或操作硬件

复现：mc02 配置目录为 `agents/tmp/10-global_refactor/class_order/mc02`，复用上方 `verify_angle_layout.py`；主机差分入口：

```sh
python agents/10-global_refactor/verify_class_order.py --baseline ba752f6
python agents/11-shooter_state/verify.py
python agents/11-shooter_state/verify_gimbal_imu.py
```

合并与一致性核对（2026-11-18）：一致性检查修正 `Melody` 限带方波与 PolyBLEP 错位注释、`GimbalDualYaw` 指令区 `is_enable` 顺序、`Steer` 舵向注释位置、`RobotUi` 委托构造空行及 `SuperCap::Config` 分组注释空行。外部分支提交 `987d002`、`f8bdfd2`，与主线 `93d1e49` 合并为 `d39e002`（解决 `Shooter17mm`、`GimbalClassic` 布局重叠，保留 `93d1e49` 摩擦轮释放阈值与 `TODO.md`），外部工作区 `../rmpp-class-order` 已删除；主工作区未提交的 `GimbalClassic` pitch 目标限位等 7 个文件原样保留未暂存状态并完成适配。合并后两板 82 个布局对象、C 板全预设 253 个 `lib/` 与 `app/` 对象、两板 19 个手写 BSP 对象真实 ARM 编译通过（0 失败 0 告警）；四组主机差分回归、原生输出与拨弹回归、云台 IMU 3289 项检查及全部修改源码格式检查通过。
