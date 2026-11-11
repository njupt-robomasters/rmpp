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
