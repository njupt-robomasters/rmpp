# 07 超级电容

状态：已并入 master，待用户审计与上车

## 记录

- `super_cap/`：容量与 ESR 在线辨识（RLS）、虚拟缓冲能量功率控制、jelly_sentry 接入

## 接口

- `lib/power/super_cap.{hpp,cpp}`；app 设置裁判上限与缓冲能量，超电给出电池侧目标与底盘功率目标
- 模式 OFF / CHARGE / BOOST；固件 100 ms 未收到命令关 DCDC
- `power_battery_ref` 由裁判上限与缓冲能量计算，所有模式下发；`power_chassis_ref` 交给底盘
- 未装超电时不调用 `send()`，按裁判缓冲能量运行

## 文档

- `../02-conventions/application.md` 的“jelly_sentry 超级电容”

## 待办

0x051 电压电流符号与量程、CHARGE 稳态 `power_cap`、BOOST 电压补偿衰减、断联重连、UI 储能百分比，详见 `../TODO.md`
