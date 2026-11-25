# 拨弹状态机与电机原生输出

## PID 输出饱和判卡弹（2026-11-20）

- 删除 Config.block_torque；FEED 用上一周期拨弹 PID 输出（成员 shoot_output，update() 末尾保存，失能为 0）与 shoot_pid_config->max_out 比较，饱和持续 block_time 后反转。阈值与输出同单位，DJI 为 A、EL05 为 N·m，三台车统一
- 纯 P 参数堵转输出为 kp×目标转速：sentry ≥4.9 Hz、leg_up ≥3.1 Hz、jelly_sentry ≥22.5 Hz 才饱和；用户将另行给拨弹 PID 加积分整定
- block_time 默认值沿用合并后的 0.2 s（合并前 0.5 s），sentry、leg_up 用默认值
- 主机测试替换独立阈值用例为：PID max_out 低于电机限幅时按 PID 饱和检测、判定周期仍按原方向输出、下一周期反转；P 输出低于 max_out 时持续前进。g++ 编译运行通过，产物在 agents/out/11-shooter_state/
- jelly_sentry、sentry、leg_up 的 shooter_17mm、gimbal_fold 与相关 app 对象 ARM 编译通过且零警告；jelly_sentry 在 agents/out/build 完整链接通过；clang-format 检查通过

## 已确认需求

- 17 mm 拨弹状态机直接在 `Shooter17mm::update()` 展开，使用 STOP、FEED、REVERSE
- 删除 `is_block`、`block_timer`；`block_elapsed`、`reverse_elapsed` 分别累加限幅后的 dt
- 拨弹 PID 输出连续饱和（达到 max_out）并持续block_time后反向运行reverse_time。STOP 清两个累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID，恢复时当周期拨弹
- 负弹频正常反转，退弹方向和速度跟随当前有符号弹频；保留拨弹独立于摩擦轮的现有修改
- 状态机先确定目标速度，再统一执行速度 PID；卡弹检测读取上一周期 PID 输出，切换到 REVERSE/FEED 后下一周期改变目标速度。2026-11-20 按用户要求由力矩目标阈值改为 PID 输出饱和
- 该次调整的 Shooter17mm 与整车应用对象编译、格式检查通过。主机复用现有状态机场景验证卡滞计时、反转、停止和 dt 限幅，通过；Zig 主机编译副本省略独立的 DM 浮点模板输出测试，日志在 `out/torque_state_compile.log`
- `Motor::set_output(float)` 返回 void，基类调用虚函数 set_current，DM 系列覆盖为 set_torque
- 云台和 17/42 mm 发射机构删除 PidOutput 配置和外部输出跟踪；PID 单位与限幅由 app 负责
- 保留显式 set_current、set_torque 及 VESC 命令、X630 静音处理

## 验证

```sh
python agents/tasks/11-shooter_state/verify.py --cxx g++ --arm-build build/leg-leg_up
```

- 主机测试通过：连续卡弹检测、未饱和帧重置计时、退弹时长、正负弹频、热量保护/失能/零弹频/离线停止、恢复重新计时、长停顿的 dt 限幅
- 零速度闭环修正验证通过：零弹频下正反速度反馈持续产生制动电流且不触发退弹，热量保护进入零速度闭环，失能时正反速度反馈均输出零电流；修正后的 `shooter_17mm.cpp` ARM 对象编译通过
- 基类引用输出分派、DM 力矩、VESC 电流报文和 X630 静音测试通过
- 19 个源码的 ARM 对象编译通过，无警告：8 个电机源码、5 个云台/发射机构源码，以及 leg_up 三个源码、infantry/sentry 云台、gimbal 示例入口
- 修改源码的 clang-format 检查及 git diff --check 通过
- 未执行完整固件链接或硬件操作；产物在 `agents/out/11-shooter_state/`，不入库

## 实车核对

sentry 双 yaw 共用一个速度 PID，DM4310 按 N·m、GM6020 按 A 接收同一输出数值；保留现有参数，待用户实车整定。leg_up 的 GM6020 pitch 现在按电流接收 PID 输出。

任务 10 的布局差分测试保留当时的输出类型接口，作为历史重构记录；本轮行为由本目录测试验证。

## 云台独立实现

- 基线 `f3b8e14`；在外部 worktree `../rmpp-gimbal-standalone`、分支 `gimbal_standalone` 完成，现已合入 master，外部 worktree 已移除
- 删除 `Gimbal` 基类与 `gimbal.{hpp,cpp}`；`GimbalClassic`、`GimbalDualYaw` 各自持有配置、电机引用、PID、指令和跨周期状态，主要流程在 `update()` 中展开
- 两类统一 `Config` 字段、`Mode` 枚举、`set_*`、`update()`、公共观测量及内部同类变量名称；示例改用 `GimbalClassic::IMU_MODE`
- 沿用主工作区已有的 `Config` 声明与实例紧邻、速度直接乘 dt 的简化；主工作区原有改动先提交为 `8a42d69`，再执行合并
- 保留模式切换、限位、参考系换算、掉线恢复、PID 和双 yaw 输出行为；双 yaw 按大 yaw 目标回写 IMU 目标的运算顺序保持原样，避免浮点误差跨周期累积
- `verify_gimbal.py` 将基线类更名后与真实当前源码一起编译，模拟相同电机反馈、指令和时钟，比较每周期目标、实测、参考系差值和电机输出

复现：

```sh
python agents/tasks/11-shooter_state/verify_gimbal.py --cxx g++
```

默认对照已提交基线；`--baseline-root <目录>` 可对照指定工作区的实际文件。本轮对照主工作区，12,000 周期、186,000 项比较通过，覆盖 ECD/IMU 模式切换、速度积分、角度设置、限位、失能恢复、反馈超时、双 yaw 单侧在线、角度回绕及 dt 上下限。

ARM 对象编译通过且无警告：两个云台源码，以及 `leg_up`、`infantry`、`sentry`、`examples-gimbal` 的全部应用源码，共 16 个对象。clang-format 与 `git diff --check` 通过；本轮未执行完整固件链接和硬件操作。

合并提交 `925e031`：解决两个基类文件的修改/删除冲突，保留其简化内容于独立类；leg_up 新增的编码器模式调用同步改为 `GimbalClassic::ECD_MODE`。合并后的主工作区对照 `8a42d69` 再次通过 12,000 周期、186,000 项比较，相关应用对象和格式检查通过；电机示例对象检查通过。日志和测试产物位于主工作区 `out/gimbal_standalone/`，外部 worktree 的对象编译日志归档至其中的 `external_worktree_logs/`。

## GimbalClassic IMU 速度闭环

已按确认方案删除 Classic ECD 模式，迁移 `infantry`、`leg_up`、`examples/gimbal` 调用；DualYaw 保持原实现。IMU 固连 pitch 随动部分，使用 ECD pitch 关节四元数将 gyro 旋转到上游 yaw 随动系，取 Z/Y 轴的绝对角速度作为反馈；电机速度仅保留观测。正负90度时 yaw 反馈分别为 `-gyro.x()`、`gyro.x()`，全程无余弦除法。角度外环可由固定配置 `use_angle_loop` 关闭，开启时采用最短四元数姿态误差的机械轴投影，保留初始化 roll 参考以连续越过竖直位置。

yaw 速度误差按用户确认的方式计算：`yaw.speed_ref - yaw.speed_measure + (yaw.speed_ref - chassis_wr)`，全部进入同一个速度 PID。pitch 按 ECD 实测保护最终原生输出，保留回程方向，截断时对速度 PID `track(0)`；IMU 目标不再受 ECD 目标钳制。

Classic 状态已展平为 `position_ref/position_measure`、`speed_ref/speed_measure`；速度目标包含目标变化率投影和角度环修正，yaw 相对速度前馈另行叠加。失能或离线时对应速度目标清零。ECD 反馈直接读取公开的 `motor`，删除重复缓存和 `imu_minus_ecd` 差值观测；步兵 UI、底盘跟随和掉头调用同步迁移。

当前验证入口：

```sh
python agents/tasks/11-shooter_state/verify_gimbal_imu.py --arm
```

- 主机编译真实 Classic、Motor、PID，仅模拟时钟和反馈；6,374 项检查通过（同步扁平字段，增加速度目标、失能清零与反馈刷新检查，移除旧差值观测断言）。覆盖整圈关节转速恢复（含正负90度）、枪口自旋和底盘运动的轴向投影、倾斜姿态下四元数外环误差下降、正负90度初始化、反号等价四元数、穿过竖直位置的外环连续性、两种配置、目标积分与 dt 限幅、角度回绕、电机速度噪声隔离、前馈叠加、pitch 双向电流/力矩保护、安装反向、积分回算及失能/断联恢复
- 真实 ARM 编译通过且无警告：Classic 源码，以及 infantry（5 个）、leg_up（3 个）、gimbal 示例（1 个）全部应用源码，共 10 个对象；日志和产物放在本任务 `out/gimbal_imu/`
- 当前 Classic 头文件、源码和主机测试源码通过同版本 clang-format 检查，相关 diff 空白检查通过；此前三处 app 接口迁移已通过格式检查
- 上节 `verify_gimbal.py` 是旧 ECD/IMU 接口的历史差分验证，复现时使用当时的提交；当前行为验证使用本节入口
- 本轮未执行完整固件链接、烧录、复位或硬件测试；上车前核对 IMU 安装轴与 ECD pitch 零点、正方向。实际实例保留原 pitch 保护范围（负15度至正30度）；反馈和四元数外环在90度处可计算，输出仍受 ECD 保护范围约束

## GimbalClassic 外环简化与 roll_ref 移除

roll_ref 用于四元数姿态误差：目标四元数缺少 roll 时，底盘倾斜带来的 roll 误差会经投影漏进 yaw 轴（yaw 机械轴在 IMU 系有 `sin(pitch)` 的 X 分量），形成稳态偏差；roll_ref 只在初始化时取一次，底盘在斜坡上转动后可达 roll 随之变化，偏差仍会出现。改为直接对 ZYX 欧拉 yaw/pitch 误差闭环，输出世界角速率，与速度指令一起经 `pitch_joint * q⁻¹` 换算到电机轴，roll 不参与，roll_ref 与目标四元数一并删除。

同时整理：`is_enable` 与在线状态集中为 `can_pitch`/`can_yaw`；`use_angle_loop` 只在一处与可控状态合并后交给私有成员 `update_angle_loop()`，它直接读写两轴状态与角度 PID，把外环修正叠加进 `speed_ref`；yaw 的 `chassis_wr` 前馈也并入 `speed_ref`，速度误差统一为 `speed_ref − speed_measure`；删除 `can_yaw_last/can_pitch_last`，不可控期间每周期同步目标即可覆盖恢复，首次 update 由 `has_update` 同步；`set_speed()` 为外部给出的电机轴速度，直接作为速度目标前馈，只有外环修正经姿态换算到电机轴；`position` 命名统一为 `angle`（`set_angle`、`angle_ref`、`*_angle_pid_config`），迁移 infantry、leg_up、examples/gimbal。

验证：`python agents/tasks/11-shooter_state/verify_gimbal_imu.py --arm`

- 主机测试重写为当前行为，3,280 项通过：水平底盘关节转速恢复与编码器噪声隔离、倾斜底盘角速度投影、倾斜底盘下速度指令保持电机轴、目标积分与 dt 限幅、yaw 回绕、两种配置、pitch 双向保护与积分回算、失能/断联恢复；新增倾斜底盘（roll ±0.25~0.3 rad）闭环运动学仿真，世界 yaw/pitch 收敛误差 < 1e-4 rad
- ARM 对象 10 个编译通过且无警告；改动文件经 `tools/format_code.py` 格式化。未执行完整固件链接与硬件测试

## GimbalClassic pitch 目标限位

问题：外环 pitch 目标无界累加，编码器保护挡住云台后目标继续累积，误差超过 π 经 `Angle` 回绕反向，云台转到另一侧限位；保护只截断输出，外环误差饱和时在限位处反复切断与满额推出，形成震荡。

修改：`update_angle_loop()` 用 `pitch.angle_measure − motor.pitch.angle` 把编码器限位平移到世界 pitch，钳制累加目标和 `set_angle()` 目标；目标被钳住且速度指令向外时撤掉 pitch 前馈；编码器输出保护保留为超调兜底。yaw 无限位，速度环模式行为不变。

验证：`python agents/tasks/11-shooter_state/verify_gimbal_imu.py --arm`

- 主机测试 3,289 项通过；新增水平底盘持续向外推杆 10 s（累计 10 rad）目标停在限位、关节收敛到限位且后 5 s 无反转、向外前馈撤除、反向指令立即离开限位，底盘前倾 0.1 rad 时上下限平移，界外目标下向内前馈保留
- `app/examples/gimbal` 已删除，脚本移除该入口；ARM 对象 9 个（infantry 5、leg_up 3、Classic 源码）编译通过且无警告。未执行完整固件链接与硬件测试
- 上车核对：持续推杆到上下限位无反转、无震荡；底盘倾斜时限位位置跟随编码器

## GimbalClassic 角度外环最大角加速度规划

问题：大阶跃目标或外力大扰动下，线性角度外环要求减速度 `kp·ω` 远超电机最大力矩对应的物理角加速度上限，减速距离不足导致超调和震荡；压低 `kp` 又会牺牲小角度响应带宽。

修改：`GimbalClassic::Config` 新增 `yaw_max_accel`、`pitch_max_accel`，默认值 `INFINITY`（`plan_angle_error()` 直接返回原误差，等效禁用规划）；设为有限正数时，在 `|e| <= e0`（`e0 = max_accel / kp²`）内保持原误差，超出后折算为 `±√(e0·(2|e| − e0))` 再送入角度 PID，使比例项沿恒减速度平方根制动曲线 `±√(2·max_accel·(|e| − e0/2))` 平滑过渡，在 `e0` 处值与一阶导数连续。

验证：`python agents/tasks/11-shooter_state/verify_gimbal_imu.py --arm`

- 主机测试 3,325 项通过；新增默认 `INFINITY` 与纯线性 PID（含 P/I/D 及 `kp = 0`）逐帧恒等、分段线性区与平方根曲线数值、`e0` 处值与斜率连续、`max_out` 限幅、`max_accel = 0` 边界，以及二阶刚体加力矩饱和（`J = 0.05 kg·m²`、`τ_max = 3 N·m`）闭环仿真：对照禁用规划时 1.5 rad 阶跃大幅过冲，启用 48 rad/s² 后阶跃响应与外力掰偏恢复均零超调收敛，小角度轨迹与禁用规划时一致
- ARM 对象 9 个（infantry 5、leg_up 3、Classic 源码）编译通过且无警告。未执行完整固件链接与硬件测试
- 上车核对：实测两轴最大角加速度后取 0.7~0.8 倍裕度填入配置，测试大角度阶跃与手动掰偏松手无过冲、小角度刚度不变
