# 全局代码优化

基线：`8f01151`（改动前已提交工作区）。长期约定已写入 `AGENTS.md`、`docs/architecture.md`、`docs/application.md`、`docs/motor.md`、`docs/bsp.md`

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
- 类布局：首个 `public:` 只放类型、常量、函数，变量另起 `public:`/`private:` 按重要性排序
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
