# 03 底盘与控制

状态：任务均已并入 master，待用户审计与上车

底盘、靶车、云台与发射机构、遥控器操控量的实现。

## 任务记录

- `quad_steer/`：四舵轮移植与矩阵底盘、统一功率控制、PID 回算、UI 下放
- `dummy/`：PR #3 靶车移植到 `app/dummy`，遥控映射、删减与 follow PID 采样
- `shooter/`：17 mm 拨弹状态机、PID 饱和判卡弹、电机原生输出、GimbalClassic 限位与角加速度规划
- `vt13/`：VT13 键盘斜坡、云台、发射与自瞄操控量解析

## 关键约定

- VT13 在 `update()` 内合成 `is_enable`、`vx`/`vy`/`wr`、`yaw_speed`/`pitch_speed`、`bullet_speed`、`bullet_freq`、`is_aim`；app 优先读这些量，原始量留在 `rc`、`mouse`、`key`
- 平移与旋转为摇杆或拨轮乘满量程，叠加键盘斜坡后限幅；云台为摇杆叠加鼠标，无斜坡
- 云台 pitch 抬头为负；自瞄在 `is_aim && is_detect` 时才放行视觉控制与弹频
- 底盘接口统一 `set_speed()`、`update()`、`send()`；详细控制说明见 `../02-conventions/application.md` 的“GimbalClassic 控制与观测”“遥控器映射”

## 上车核对

各任务 README 的“上车核对/实车核对”一节，汇总见 `../TODO.md`
