# 拨弹状态机与电机原生输出

## 已确认需求

- 17 mm 拨弹状态机直接在 `Shooter17mm::update()` 展开，使用 STOP、FEED、REVERSE
- 删除 `is_block`、`block_timer`；`block_elapsed`、`reverse_elapsed` 分别累加限幅后的 dt
- 连续命令电流饱和达到 block_time 后反向运行 reverse_time；STOP 清两个累计时间，使能时保持零速度闭环，失能时输出零电流并清 PID，恢复时当周期拨弹
- 负弹频正常反转，退弹方向和速度跟随当前有符号弹频；保留拨弹独立于摩擦轮的现有修改
- 状态机先确定目标速度，再统一执行速度 PID；卡弹检测读取上一周期命令电流，切换到 REVERSE/FEED 后下一周期改变目标速度，主机测试已同步并通过
- `Motor::set_output(float)` 返回 void，基类调用虚函数 set_current，DM 系列覆盖为 set_torque
- 云台和 17/42 mm 发射机构删除 PidOutput 配置和外部输出跟踪；PID 单位与限幅由 app 负责
- 保留显式 set_current、set_torque 及 VESC 命令、X630 静音处理

## 验证

```sh
python agents/tasks/11-shooter_state/verify.py --cxx g++ --arm-build build/leg_up
```

- 主机测试通过：连续卡弹检测、未饱和帧重置计时、退弹时长、正负弹频、热量保护/失能/零弹频/离线停止、恢复重新计时、长停顿的 dt 限幅
- 零速度闭环修正验证通过：零弹频下正反速度反馈持续产生制动电流且不触发退弹，热量保护进入零速度闭环，失能时正反速度反馈均输出零电流；修正后的 `shooter_17mm.cpp` ARM 对象编译通过
- 基类引用输出分派、DM 力矩、VESC 电流报文和 X630 静音测试通过
- 19 个源码的 ARM 对象编译通过，无警告：8 个电机源码、5 个云台/发射机构源码，以及 leg_up 三个源码、infantry/sentry 云台、gimbal 示例入口
- 修改源码的 clang-format 检查及 git diff --check 通过
- 未执行完整固件链接或硬件操作；产物在 out/，不入库

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
