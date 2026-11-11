# 靶车 app/dummy

来源：PR #3（Joushua233/rmpp master，`rmpp/app/target`），按当前框架重写，不保留 git 历史

## 复现

```sh
git fetch origin pull/3/head:refs/remotes/pr/3
cmake --preset dummy
cmake --build build/dummy --parallel 8
```

## 映射

- 底盘：四个 N630（C620 协议）用 `X630` 驱动，CAN1 0x201–0x204，0x200 组帧；直接用 `ChassisOmniSpeed`，跟随 PID 由库完成
- HFI 噪音：由 `X630` 滞回静音处理（见 `docs/motor.md`），dummy 用默认阈值
- 已删：直线防偏航、航向零点、`rc_enable()`、`is_gimbal_mode()`
- 云台：IMU（C 板）在底盘上，云台世界系朝向 = `attitude.yaw + yaw.position`，单级 P 闭位置环输出电流；未使能时目标跟随测量
- 电机：RMD-L-7025 直连，`lib/motor/rmdl7025`，0x141/0x241；`gimbal_loop` 按 2 ms 限频调用 `send()`，两种报文合计 500 Hz
- CAN 发送分别在 `chassis_loop`、`gimbal_loop` 内；已去掉摇杆额外 8% 死区和反馈轮速死区
- 遥控：FSi6X Uart(3)；SWA 下使能；SWC 空闲
  - SWD 下：60 rpm 小陀螺，优先于跟随；以云台为前方平移，左摇杆水平控制云台
  - SWD 非下且 SWB 中/下：底盘跟随云台，以云台为前方，左摇杆水平控制云台
  - SWD 非下且 SWB 上：不跟随，底盘原始前方，左摇杆水平控制底盘旋转，云台保持世界系朝向
  - 云台离线：不跟随，平移以底盘原始前方
- 超电：`SuperCap` CAN1，经 `chassis` Config 传入；SWC 上 MONITOR、中 CHARGE、下 BOOST，遥控断联 OFF；无裁判系统，功率上限与缓冲能量写死 60 W / 60 J
- IMU：QuaternionEkf，实例与更新归云台（`gimbal.hpp`、`gimbal_loop`），零偏沿用 PR 值

## 删减

- PR 的 Ozone volatile 调试变量、测试模式 0/1–4/11–14/30、自写 SBUS 解析和 UART3 重配（板卡 CubeMX 已是 SBUS 参数）、CAN 包间忙等
- PR 对 GM6020/Motor/Chassis_DualSteer/unit 的公共库改动不属于靶车，未合入

## 待上车核对

- yaw 电机零点 `install.offset`：使 `yaw.position` = 云台相对底盘前方角度，逆时针为正（与 `attitude.yaw` 同向）；不对时跟随、小陀螺平移方向和世界系闭环都会偏
- `yaw_pid.kp`（1 A/10°）与 `follow_pid`（kp 5，限 360°/s）为占位值，跟随方向（符号）待实车确认；HFI 阈值 1 A/2 A/10 rpm 待实测
- 轮序与方向；7025 电机 ID、电流方向（正电流应使 `yaw.position` 增大）
- 摇杆死区：库默认 5%，PR 自写解析也是 5%；FSi6X 拨杆按精确值 240/1024/1807 判定，PR 用区间

## follow PID 微分尖峰实测（2026-11-11）

- 用户授权烧录当前 dummy；`cmake --build build/dummy --parallel 8` 返回无待构建项。OpenOCD 烧录、校验成功并复位运行，此后只读采样，未暂停 MCU
- `capture_follow_pid.py` 使用 OpenOCD TCL 6666 端口读取 SRAM；先逐段比对 Flash 与当前 ELF。变量地址来自本次 `build/dummy/rmpp.elf`，更换固件后须重新核对
- 原始数据及分析输出：`out/follow_pid/20261111_180613.csv`、同名 `.json`、`.analysis.json` 和 `.png`；输出不入库
- 第一轮 `mode=DETACH_MODE`，follow PID 无输出；按 SWA 下、SWB 下、SWD 非下重新操作后捕获到微分尖峰
- 实际参数 `kp=10`、`ki=0`、`kd=2`、`max_out=6.2831855 rad/s`，`err_fc=d_fc=0`
- `gimbal_loop()` 每至少 5 ms 发送一次；`RMDL7025::send()` 交替发送 0xA1 与 0x94。0xA1 更新电流/速度，0x94 更新角度，两种回复都调用公共回调。因此回调间隔约 5～6 ms，角度更新约 10～12 ms
- follow PID 约每 1 ms 对保持后的角度做差分；新角度到达时，一次输出集中承载约十个控制周期的变化，随后 D 回到零。低频绘图看起来平滑的误差包含这种台阶
- 捕获到 D=+165.2861，dt=1.001036 ms，对应误差台阶 +4.74°；另一个 D=-123.0864、dt=1.001089 ms，对应 -3.53°，均与 0.01° 协议刻度一致。本轮跟随误差在约 [-1.476, 1.681] rad，尖峰发生在单圈角度边界以内
- SWD 采样在跟随段约 53 Hz，变量读取期间 MCU 持续运行；分析脚本剔除 P/误差或输出和限幅明显不一致的样本。数据展示已捕获的尖峰，不覆盖每个 1 kHz 控制周期
- 建议先明确跟随 D 输入：当前 err=yaw.position，对应角速度为 +yaw.speed，可利用现有 `Pid::calculate(err, derr)` 显式传入；也可按实际带宽设置 D 低通。工程代码和 PID 参数均未修改

复现分析：

```sh
python agents/tasks/05-dummy_target/analyze_follow_pid.py agents/tasks/05-dummy_target/out/follow_pid/20261111_180613.csv
```
