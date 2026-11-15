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
