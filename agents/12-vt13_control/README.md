# VT13 操控量解析

## 目标

VT13 原始量收进 `rc`，在 `update()` 内合成 `is_enable`、`vx`/`vy`/`wr`、`yaw_speed`/`pitch_speed`、`bullet_speed`、`bullet_freq`、`is_aim`；leg_up、sentry 改用这些操控量，sentry 移除 FSi6X。长期结论见 `docs/application.md`“遥控器映射”

## 复现

```sh
python -I agents/12-vt13_control/test_vt13.py
python -I agents/04-leg_chassis/test_leg_up_control.py
```

`test_vt13.py` 用 Timer、Uart 替身编译实际 `lib/rc/vt13.cpp` 与 CRC，经串口回调送入完整帧，覆盖帧解析、键盘斜坡（反向清零、松手减速、与摇杆叠加限幅）、拨轮与鼠标限幅、trigger 改弹频、鼠标左键开火、app 修改 `config.bullet_freq`、S/C 挡、自瞄、断联清零与重连

## 行为变化

- sentry
  - 删除 FSi6X 实例与全部拨杆操作（使能、fn、校准组合、射击、自瞄）
  - 删除鼠标角加速度限制，鼠标与摇杆直接叠加
  - C、Z 键改为修改 `vt13.config.bullet_freq`（5–25 发/秒），UI 显示该值
  - 射击条件统一按 `vt13.is_aim`（photo 或右键）要求视觉开火许可；原来遥控器射击只看 photo、鼠标射击只看右键
  - pitch 摇杆与鼠标方向按 Classic 云台约定（抬头为负），与原 sentry 方向相反
- leg_up：新增键鼠操控（WASD 中仅 W/S 生效、Shift/Ctrl 旋转、鼠标云台、左键射击）；键盘加速度 2 m/s²、减速度 4 m/s²、角加减速度 720 °/s² 为暂定值

## 上车核对

- sentry pitch 摇杆与鼠标方向；若方向反了，说明 DualYaw 云台与 Classic 的 pitch 正方向不同，需要在 app 取反
- 鼠标 pitch 方向（leg_up、sentry）
- leg_up 键盘加减速度对平衡的影响
