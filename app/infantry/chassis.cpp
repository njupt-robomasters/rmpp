#include "app.hpp"
#include "bsp/timer.hpp"
#include <algorithm>

// 操控参数
static constexpr float VXY_MAX = 5.0f;     // 平移最大速度，75W下2.4m/s，不限功率6.3m/s
static constexpr float WR_MAX = 150 * rpm; // 旋转最大角速度，75W下125rpm，不限功率240rpm
static constexpr float AXY = 100.0f;       // 键盘平移加速度，m/s^2
static constexpr float DXY = 100.0f;       // 键盘平移减速度，m/s^2
static constexpr float AR = 1000 * rpm;    // 键盘旋转角加速度，rad/s^2
static constexpr float DR = 1000 * rpm;    // 键盘旋转角减速度，rad/s^2

// 跨周期状态
static Timer loop_timer;
static bool is_fn = false;
static float key_vx = 0, key_vy = 0, key_wr = 0;

// 按住正向或反向键时加速，松开时减速到 0
static float key_speed(const float speed, const bool positive, const bool negative, const float maximum, const float acceleration, const float deceleration, const float dt) {
    if (positive)
        return std::clamp(std::max(speed, 0.0f) + acceleration * dt, -maximum, maximum);
    if (negative)
        return std::clamp(std::min(speed, 0.0f) - acceleration * dt, -maximum, maximum);
    if (speed > 0)
        return std::max(0.0f, speed - deceleration * dt);
    return std::min(0.0f, speed + deceleration * dt);
}

void chassis_loop() {
    const float dt = loop_timer.restart();
    chassis.set_enable(rc_enable());

    // fn 跟随模式：FSi6X 优先取 SWA 下；VT13 在 N、S 挡时 fn 键置位、pause 键清除
    if (fsi6x.is_connect) {
        is_fn = fsi6x.swa == FSi6X::DOWN;
    } else if (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) {
        if (vt13.fn)
            is_fn = true;
        if (vt13.pause)
            is_fn = false;
    } else {
        is_fn = false;
    }
    chassis.set_mode(is_fn ? ChassisMatrix::FOLLOW_MODE : ChassisMatrix::DETACH_MODE);

    // 键盘：WASD 平移，Shift、Ctrl 旋转；断联时直接清零
    if (vt13.is_connect) {
        key_vx = key_speed(key_vx, vt13.key.w, vt13.key.s, VXY_MAX, AXY, DXY, dt);
        key_vy = key_speed(key_vy, vt13.key.a, vt13.key.d, VXY_MAX, AXY, DXY, dt);
        key_wr = key_speed(key_wr, vt13.key.shift, vt13.key.ctrl, WR_MAX, AR, DR, dt);
    } else {
        key_vx = key_vy = key_wr = 0;
    }

    // 摇杆与键盘叠加，断联时摇杆已复位为 0
    const float vx = (vt13.x + fsi6x.x) * VXY_MAX + key_vx;
    const float vy = (vt13.y + fsi6x.y) * VXY_MAX + key_vy;
    const float wr = vt13.wheel * WR_MAX + key_wr;
    chassis.set_speed(std::clamp(vx, -VXY_MAX, VXY_MAX), std::clamp(vy, -VXY_MAX, VXY_MAX), std::clamp(wr, -WR_MAX, WR_MAX));

    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power.chassis_ref);
    chassis.update();
}
