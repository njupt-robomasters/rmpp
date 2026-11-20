# MiniPC 导航与 pitch1 失能

2026-11-20 用户授权在当前工作区实施。模块说明见 `../../docs/minipc.md`。

## 实现范围

- `lib/minipc/Nav` 接入 17 字节小端 float32 下行协议，独立 MODBUS CRC16，任意分片与坏帧重同步；有效帧才刷新超时
- 中断保存待发布帧，主循环 `update()` 通过短临界区取得完整快照；默认 100 ms 离线清空指令
- jelly_sentry 使用 CDC1 自瞄、CDC2 导航。flag=0 底盘随 IMU 头，flag=1 关闭随头并启用原有小陀螺；人工底盘输入接管，大小 yaw 离线停止自主运动
- 雷达系到云台系使用 `R(-q2)`，底盘内部再用 `R(q1+q2)` 转到底盘系，合成 `R(q1)`；雷达安装轴按大 yaw 对齐处理
- pitch1 全程失能，set_enable/update 均固定关闭，清空两级 PID、撤除该轴闭环输出计算并写零原生输出；4 ms 调度保留，实际 DmMotor::send() 发失能帧
- C 板 sentry 同步迁移 Nav 构造与速度读取，保留原有底盘控制策略；两处应用删除未发送到上位机的目标点/返航占位赋值
- yaw 字段解析并保存，具体控制用途待上位机约定；自瞄协议和 MCU 上行协议后续接入

## 复现

测试工具共用 `agents/.tools/`，缓存、可执行文件、编译日志和 ARM 对象生成在本任务 `out/`，可在验证后清理：

```powershell
python -m pip install --target agents/.tools ziglang==0.15.2 clang-format==22.1.0
python agents/tasks/13-minipc_nav/verify.py --arm
```

格式化与检查使用 `tools/format_code.py --clang-format agents/.tools/clang_format/data/bin/clang-format.exe`，路径限定为本轮修改的 9 个手写源码文件。

## 验证结果

- Nav 与 pitch1：90,578 项检查通过。覆盖独立 CRC 查表/黄金帧、首帧逐字节、全部分片位置、粘包、每个位置插字节/丢字节、噪声恢复、CRC 错误、NaN/Inf/非法 flag、坏帧超时、重连、重复状态、CDC1/2/3 隔离
- pitch1 使用真实 GimbalFold、Motor、PID、DM4310/DmMotor，6000 周期覆盖总使能、模式切换、反馈超时/恢复、非零重力补偿配置和主动注入非零目标；每周期验证失能、current_ref=torque_ref=0，并检查真实驱动发出 `FF FF FF FF FF FF FF FD`
- 生产底盘流程：104 项检查通过。原样复制 chassis.cpp 到 out，仅替换 app.hpp 的设备依赖；覆盖随头/小陀螺、负小 yaw 角转换、21 组两级转换合成、超量程输入限幅、人工接管/释放、超时、yaw 离线、整车失能
- 19 个 ARM 对象真实编译通过，零警告；覆盖 jelly_sentry、sentry、jelly_sentry_debug-gimbal、jelly_sentry_debug-gimbal_motor 的应用源码及 Nav。命令来自各目标 compile_commands.json，输出重定向至本任务 out，无完整固件链接
- 主机测试编译的兼容警告来自第三方 Eigen 对 libc++ 已弃用浮点元信息的引用，日志保存在 `out/*_compile.log`
- 本轮未烧录、复位、halt 或改写硬件；用户审计后更新固件，核对 CDC 枚举、雷达安装偏角、实际随头方向及 pitch1 失能状态

ARM 编译摘要由脚本生成至 `out/arm_summary.json`。2026-11-20 按用户要求清理了各任务 out 中的缓存、日志和测试产物，工具合并至 `agents/.tools/`；上面的验证结果保留，后续重跑脚本可重新生成产物与摘要。
