# 上游合并预检

2026-11-15：只检查和规划，尚未执行 merge、stash、暂存、提交或修改工程代码

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

## 后续顺序

1. 用户明确要求开始后，先独立保存、核对本地提交和未提交成果，包括生成文件及未跟踪脚本；不擅自提交或丢弃
2. 合并上游，处理预设与文档冲突；用户决定 CAN2 配置并处理 .ioc/生成文件
3. 迁移 EL05、GimbalFold、整车及全部相关调试入口到上游接口，保留实际调参
4. 对涉及的手写代码格式检查、局部真实编译；单独列出待硬件验证项

本轮未构建、烧录、复位或进行硬件操作；尚不能声称合并后可编译或可上车
