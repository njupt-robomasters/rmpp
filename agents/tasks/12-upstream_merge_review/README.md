# 上游合并与接口适配

2026-11-15：按用户要求先提交当前成果，再合并上游、启用 CAN2 自动重发并迁移接口。以下预检基线保留用于复核，实施结果见末节

## 基线与复现

- 本地 master：`542dcc254abafc052467cb22bb2b1906a2aa10f8`
- 已执行 `git fetch origin`，上游 origin/master：`f3b8e14687308db0b2cbcb8b1057bd2c6c77dbce`
- 共同祖先：`564c678d5e6714a4c35aaf54d4033d68a7d8d378`
- 本地独有 1 个提交，上游独有 26 个提交；暂存区为空
- 原有未提交内容差异为 23 个已跟踪文件；大量 Drivers/CMake 文件虽显示 M，Git 规范化后的 diff 无内容差异，不认定为驱动升级
- 未跟踪文件与上游文件没有同路径覆盖；未纳入临时合并快照
- 运行 `node agents/tasks/12-upstream_merge_review/review.cjs` 复现，先按需 fetch；结果保存在被忽略的 `out/`
- 脚本使用独立裸仓库、索引与对象库，通过 alternates 只读引用工程对象；不移动工程分支或改写工程索引

## 文本冲突

只合并已提交 HEAD 时，只有 `CMakePresets.json` 冲突。将当前已跟踪文件的未提交改动一起纳入后，共 4 个冲突文件：

| 文件 | 内容与处理方向 |
| --- | --- |
| `CMakePresets.json` | 本地插入 jelly_sentry 与调试预设，上游把 leg 拆成 leg_down/leg_up；需要同时保留本地预设和上游拆分结果 |
| `agents/TODO.md` | 两侧新增任务记录，按现状整合 |
| `agents/docs/motor.md` | 本地 EL05 MIT 说明与上游电机接口说明，需随接口迁移同步更新 |
| `boards/mc02/rmpp.ioc` | 本地 CAN2 AutoRetransmission=DISABLE，上游 ENABLE 并增加 FIFO 模式；由用户决定 CAN2 参数并操作 CubeMX |

`boards/mc02/Core/Src/fdcan.c` 可自动合并，但须核对其实际初始化与最终 .ioc 一致。其本地改动仅末尾空行，上游修改了 CAN 初始化行为

## 自动合并不能解决的接口不兼容

以下根据实际本地调用与上游声明逐项对照确认，尚未运行编译：

- `EL05`：本地使用 `Motor(config)`、`config.bus/drive/install`、`raw.*`、`is_enable_`；上游改为带型号默认参数的构造、平铺 Config、`raw_current/raw_torque/raw_speed/raw_position`、`is_enable`
- 上游 Shooter17mm 经 `Motor::set_output()` 输出。EL05 应像达妙驱动一样覆盖为力矩输出；否则继承的默认电流语义会改变原 PID 输出单位
- `GimbalFold`：本地依赖旧 Gimbal 构造、`PidOutput`、带尾随下划线的基类状态、旧虚函数签名及零点存取；上游 Gimbal 已移除相关接口，`backward(float dt)`、`update(imu_yaw, imu_pitch)` 需要迁移
- 本地 PID 调用没有 dt，上游 `calculate(dt, err, derr)` 要求持有方提供周期；折叠云台和单轴/拨弹调试入口均受影响
- `app/jelly_sentry` 和 `app/jelly_sentry_debug` 的电机初始化需迁移；上游移除了型号 `KT` 常量、重组底盘配置，不能只修云台和 EL05
- 本地实测安装方向、PID 参数、CAN ID、可写发送周期和临时注释的发送代码均是既有调试成果，迁移时逐项保留并单独审计

## 实施结果

- 保存提交：`4d59574`，包含既有工程改动、生成文件和任务脚本。本机未跟踪 `.agents/`、`.clangd`、`.settings/` 保留在工作区，不加入工程提交
- 合并上游 `f3b8e14`。提交预检记录后 `agents/README.md` 也发生任务索引冲突，实际共 5 个文件；两侧任务与索引均保留
- CMakePresets 同时保留 jelly_sentry 与 10 个调试入口、上游 leg_down/leg_up；移除已由上游淘汰的 leg 预设
- 用户明确指定采用上游 CAN2 配置，冲突段取上游 ENABLE 和 FIFO；保留本地 CubeMX 版本等其他改动。实际 `fdcan.c` 自动合并得到 ENABLE/FIFO，与 .ioc 一致，未调用 CubeMX 重新生成
- EL05 采用平铺配置、原始反馈标量与原生力矩输出，保留 MIT 帧、周期使能与清错逻辑；app 显式配置 5 Nm 和 `5/0.94` A，电流/力矩比例保持原值
- GimbalFold 通过 `backward(dt)` 统一传入 PID 周期；IMU 更新、yaw 零点与持久化移至 app，移除旧输出类型和旧基类接口依赖；保留独立轴 PID、重力补偿与电机限幅跟踪
- 底盘改由 app 更新超电并设置 power_chassis_ref；VESC 显式保留原参数 20 A、20 Nm、reduction=7
- 裁判数据中已有 hurt_data、hurt_sequence、shoot_data 恢复公开，继续支持本地受击方向 UI 与实测弹速，其他字段权限保持上游状态
- 实机 PID、安装方向、零点、CAN ID、发送周期及被注释的发送代码保留；删除的接口职责已迁至 app 或电机原生输出

## 验证与复现

工具：Arm GCC 14.3.1、CMake 4.4、Ninja 1.13；Python 3.12 路径为 `C:/Users/joushua/.local/bin/python3.12.exe`

```powershell
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/tasks/12-upstream_merge_review/verify_baseline.py
& 'C:/Users/joushua/.local/bin/python3.12.exe' agents/tasks/12-upstream_merge_review/compile.py
```

- 29 项基线/配置检查：PID 数值、发送周期、IMU 安装和俯仰零点/限位、11 个入口的实际发送开关、预设与 CAN2 配置一致性
- 整车 rmpp_app、rmpp_lib、rmpp_bsp 及 FDCAN 初始化对象局部编译通过；10 个 jelly_sentry_debug 入口 rmpp_app 对象全部编译通过，零警告
- clang-format 22.1.0 格式化并检查本次 16 个手写源码文件，`git diff --check` 检查任务改动
- 编译命令与日志保存在 `out/compile_summary.json`、`out/*_compile.log`，工程对象产物仍使用默认 `build/`
- 本轮仅对象编译与静态基线核对；未完整链接、运行控制算法测试或操作硬件。CAN2 重发/FIFO 的实际负载、7025 干扰与 EL05/4310 反馈需用户审计后上板核对

## 合入上游 09766fa（2026-11-18）

- 本地基线 `a220339`。继续工作区已开始的合并，保留两侧 TODO 和上游单份裁判配置，并保留折叠哨兵读取的受击记录、计数及实测弹速
- 上游删除云台基类后，将 `GimbalFold` 移入 `app/jelly_sentry`。四轴控制独立执行，整车和两种云台调试目标共用实现。保留实机调参、CAN 绑定与发送调度
- 迁移裁判在线字段 `is_connect`，移除与已删除示例对应的 `examples-gimbal` 预设
- `python agents/tasks/12-upstream_merge_review/verify_gimbal_fold.py` 对比 `a220339`。本机用 Zig 0.15.2 的 `zig c++`，通过 `--cxx <zig.exe> c++` 指定。6,000 周期、144,000 项比较通过，覆盖模式切换、掉线恢复、限位、重力补偿和四轴电流与力矩输出。主机编译提示第三方 Eigen 和 GCC 专用 pragma 的兼容警告
- `python agents/tasks/06-super_cap_capacitance/verify_jelly_sentry.py` 的 750 项超电策略检查通过
- Arm GCC 局部编译通过，覆盖公共库及 jelly_sentry、三个相关调试目标、sentry、infantry、leg_up、examples-motor、test。两个云台调试目标对象部分链接后无未解析的 GimbalFold 符号。格式与 diff 检查通过，未做完整固件链接或硬件验证
