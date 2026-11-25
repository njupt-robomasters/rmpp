# MiniPC 导航与 pitch1 失能

2026-11-25 已迁移到上游 A3/A4 导航接口。CDC2 以 5 ms 周期上报在线大 yaw 的有限实测角，目标坐标默认零。当前协议见 [对接文档](../../../docs/minipc_protocol.md)，以下日期记录保留各版本当时的行为。


2026-11-20 用户授权在当前工作区实施。模块说明见 `../../docs/minipc.md`。

## 实现范围

- `lib/minipc/Nav` 接入 17 字节小端 float32 下行协议，独立 MODBUS CRC16，任意分片与坏帧重同步；有效帧才刷新超时
- 中断保存待发布帧，主循环 `update()` 通过短临界区取得完整快照；默认 100 ms 离线清空指令
- jelly_sentry 使用 CDC1 自瞄、CDC2 导航。有效 flag=0 关闭随头并以 60 rpm 自动小陀螺旋转，flag=1 底盘随 IMU 头；人工平移接管速度并保持对应旋转模式，人工主动旋转优先，大小 yaw 离线停止自主运动
- 人工与导航平移均以最终 IMU 头为前方，导航速度直接传入，底盘内部用总云台角 `R(q1+q2)` 转到底盘系；Fn 仅启用自主模式
- pitch1 全程失能，set_enable/update 均固定关闭，清空两级 PID、撤除该轴闭环输出计算并写零原生输出；4 ms 调度保留，实际 DmMotor::send() 发失能帧。flag=0 抬头、flag=1 折叠仅记录 TODO
- C 板 sentry 同步迁移 Nav 构造与速度读取，保留原有底盘控制策略；两处应用删除未发送到上位机的目标点/返航占位赋值
- 下行 yaw 字段解析并保存，具体控制用途待上位机约定；自瞄协议已接入，见任务 14

## 大 yaw 上报（2026-11-24）

- `Nav::update(float yaw = 0, bool is_yaw_connect = false)` 按现有 Aim 的计时、编码和 CRC 写法，内部判断在线并每 5 ms 发送 8 字节大 yaw 相对底盘弧度角。jelly_sentry 仅在原 update 调用处传入实测角和在线状态；协议见模块文档
- 按用户要求保持最小改动，无参数调用维持仅接收行为；新增发送检查放在 `agents/out/15-minipc_nav/nav_telemetry_test.cpp`，原主机测试仅补充 CDC 发送桩
- 发送检查通过，覆盖黄金帧、弧度、200 Hz、非有限值及忙后恢复；精简后原 Nav/pitch1 的 90,578 项及底盘 51 项回归通过，app.cpp/nav.cpp ARM 对象编译零警告，3 个业务文件格式检查通过。未完整链接或操作硬件

## 复现

测试工具共用 `agents/.tools/`，缓存、可执行文件、编译日志和 ARM 对象生成在本任务 `agents/out/15-minipc_nav/`，可在验证后清理：

```powershell
python -m pip install --target agents/.tools ziglang==0.15.2 clang-format==22.1.0
python agents/tasks/15-minipc_nav/verify.py --arm
```

格式化与检查使用 `tools/format_code.py --clang-format agents/.tools/clang_format/data/bin/clang-format.exe`，路径限定为本轮修改的 9 个手写源码文件。

## 验证结果

- 2026-11-23 本次调整仅运行底盘流程测试，51 项检查通过，覆盖头坐标输入、两种 flag、人工平移保留旋转模式、人工旋转优先和超时停止；`chassis.cpp`、`gimbal.cpp` ARM 对象编译与三个修改源文件格式检查通过。主机编译有 4 条 Eigen/C++23 弃用警告；未完整构建或操作硬件

- Nav 与 pitch1：90,578 项检查通过。覆盖独立 CRC 查表/黄金帧、首帧逐字节、全部分片位置、粘包、每个位置插字节/丢字节、噪声恢复、CRC 错误、NaN/Inf/非法 flag、坏帧超时、重连、重复状态、CDC1/2/3 隔离
- pitch1 使用真实 GimbalFold、Motor、PID、DM4310/DmMotor，6000 周期覆盖总使能、模式切换、反馈超时/恢复、非零重力补偿配置和主动注入非零目标；每周期验证失能、current_ref=torque_ref=0，并检查真实驱动发出 `FF FF FF FF FF FF FF FD`
- 生产底盘流程：104 项检查通过。原样复制 chassis.cpp 到 out，仅替换 app.hpp 的设备依赖；覆盖随头/小陀螺、负小 yaw 角转换、21 组两级转换合成、超量程输入限幅、人工接管/释放、超时、yaw 离线、整车失能
- 19 个 ARM 对象真实编译通过，零警告；覆盖 jelly_sentry、sentry、jelly_sentry_debug-gimbal、jelly_sentry_debug-gimbal_motor 的应用源码及 Nav。命令来自各目标 compile_commands.json，输出重定向至本任务 out，无完整固件链接
- 主机测试编译的兼容警告来自第三方 Eigen 对 libc++ 已弃用浮点元信息的引用，日志保存在 `agents/out/15-minipc_nav/*_compile.log`
- 本轮未烧录、复位、halt 或改写硬件；用户审计后更新固件，核对 CDC 枚举、雷达安装偏角、实际随头方向及 pitch1 失能状态

ARM 编译摘要由脚本生成至 `agents/out/15-minipc_nav/arm_summary.json`。2026-11-20 按用户要求清理了各任务 out 中的缓存、日志和测试产物，工具合并至 `agents/.tools/`；上面的验证结果保留，后续重跑脚本可重新生成产物与摘要。
