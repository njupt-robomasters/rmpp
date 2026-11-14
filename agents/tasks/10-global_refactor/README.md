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

`out/` 下为一次性编辑脚本，不入库

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
python agents/tasks/10-global_refactor/layout_regression.py --baseline 795e263
```

- 应用对象编译：dummy、examples-attitude/cdc/gimbal/kv_store/melody/motor/rc/referee/rgb、leg_down、leg_up、sentry、test，共 14 个预设
- 公共库与手写 BSP：dummy 覆盖 C 板，examples-cdc 覆盖 mc02
- 主机差分回归：layout_motor_regression.cpp 验证八类电机反馈、限幅、零点、离线处理、CAN 命令及 17 mm 发射机构电流/力矩输出；sim.cpp 验证超电容量/ESR；power_sim.cpp 验证功率与虚拟缓冲能量；layout_melody_regression.cpp 比较直驱音符时序、带伴奏和纯旋律的复音浮点采样。分别编译基线与当前实现，要求输出逐字节一致
- `layout_regression.py` 只读取指定 Git 基线，提取文件与构建产物放在本任务 out/layout_regression；包含生成和第三方目录的工程文件未改动
- 保留的应用告警：motor 示例两个未使用函数；leg_up/gimbal.cpp 的 is_auto_aim 未使用
- 构建日志和差分结果位于 out/；最终状态以本节对应执行结果为准，早期失败尝试也保留在 out/ 便于追溯

待用户审阅 SWD 展开顺序与接口后决定合并、上车测试。

轮腿补充调整：力映射角度按实际顶点命名为 ∠OCB（cos_ocb、tan_ocb）；设置接口统一为 set_allow_rescue(allow_rescue)、set_trigger_jump(trigger_jump)、set_leg_length(leg_length)，同步 leg_down 调用点；update_kin 合入 update，保留原执行顺序。轮腿源码与 leg_down 的 chassis.cpp、app.cpp 对象编译通过，格式及 diff 检查通过。
