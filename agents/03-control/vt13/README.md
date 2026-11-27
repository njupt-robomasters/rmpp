# VT13 操控量解析

状态：已并入 master，待用户审计与上车

## 目标

VT13 原始量收进 `rc`，在 `update()` 内合成 `is_enable`、`vx`/`vy`/`wr`、`yaw_speed`/`pitch_speed`、`bullet_speed`、`bullet_freq`、`is_aim`；leg_up、sentry 改用这些操控量，sentry 移除 FSi6X。长期结论见 `agents/02-conventions/application.md`“遥控器映射”

## 复现

```sh
python -I agents/03-control/vt13/test_vt13.py
python -I agents/04-leg/chassis/test_leg_up_control.py
```

`test_vt13.py` 用 Timer、Uart 替身编译实际 `lib/rc/vt13.cpp` 与 CRC，经串口回调送入完整帧，覆盖帧解析、键盘斜坡（反向清零、松手减速、与摇杆叠加限幅）、拨轮与鼠标限幅、trigger 改弹频、鼠标左键开火、鼠标滚轮调整弹频（由 0 上跳加、下跳减、同号保持不重复、限幅）、app 修改 `config.bullet_freq`、S/C 挡、自瞄、断联清零与重连、比赛模式断联锁存挡位

## 断联

超时后一律 `is_connect = false`，摇杆、拨轮、鼠标、键盘、按键与键盘斜坡全部复位，操控量随之为 0。挡位按 `is_in_game` 分两种：

- 非比赛（默认 `set_in_game(false)`）：挡位一并复位为 `ERR`，`is_enable` 与 `bullet_speed` 归零，断联就是断联
- 比赛（`set_in_game(true)`）：只保留 `rc.mode`，使能和摩擦轮沿用最后一次有效帧；收到新的有效帧后才按实测更新

比赛中保留挡位只为现场链路抖动时不丢使能和摩擦轮状态；`rc` 中的其他按键照常复位，所以 fn 锁存、自瞄、校准组合键在断联后都不会误保持。app 每周期在 `update()` 前用 `game_progress == GAMING` 设置，两台哨兵与 leg_up 相同。上电后从未收到有效帧时挡位仍为 `ERR`，不使能

## 行为变化

- sentry
  - 删除 FSi6X 实例与全部拨杆操作（使能、fn、校准组合、射击、自瞄）
  - 删除鼠标角加速度限制，鼠标与摇杆直接叠加
  - 鼠标滚轮调整弹频：`Config` 的 `bullet_freq` 为默认值，滚轮在 `[bullet_freq_min, bullet_freq_max]` 内按 `bullet_freq_step` 增减，默认 5~15 发/秒、步进 5；UI 弹频数值和 BULLET 弧显示该值
  - 射击条件统一按 `vt13.is_aim`（photo 或右键）要求视觉开火许可；原来遥控器射击只看 photo、鼠标射击只看右键
  - pitch 摇杆与鼠标方向按 Classic 云台约定（抬头为负），与原 sentry 方向相反
- leg_up：新增键鼠操控（WASD 中仅 W/S 生效、Shift/Ctrl 旋转、鼠标云台、左键射击）；键盘加速度 2 m/s²、减速度 4 m/s²、角加减速度 720 °/s² 为暂定值

## 上车核对

- sentry pitch 摇杆与鼠标方向；`pitch_speed` 与 GimbalClassic、GimbalDualYaw、GimbalFold 同为低头为正，app 直接传入，不再取反；上车方向仍相反时先查 IMU 安装方向
- 鼠标 pitch 方向（leg_up、sentry）
- leg_up 键盘加减速度对平衡的影响
