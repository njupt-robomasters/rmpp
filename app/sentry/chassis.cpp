#include "app.hpp"
#include "bsp/timer.hpp"
#include "math/math.hpp"
#include <algorithm>
#include <cmath>

// 操控参数
static constexpr float VXY_MAX = 5.0f;     // 平移最大速度，100W下2.3m/s，不限功率3.2m/s
static constexpr float WR_MAX = 120 * rpm; // 旋转最大角速度，100W下107rpm，不限功率155rpm
static constexpr float WR_MIN = 60 * rpm;  // 导航满速平移时的小陀螺角速度
static constexpr float AXY = 100.0f;       // 键盘平移加速度，m/s^2
static constexpr float DXY = 100.0f;       // 键盘平移减速度，m/s^2
static constexpr float AR = 1000 * rpm;    // 键盘旋转角加速度，rad/s^2
static constexpr float DR = 1000 * rpm;    // 键盘旋转角减速度，rad/s^2

// 自主导航参数
static constexpr bool FORCE_GAME_POSITION = false;
static constexpr bool FORCE_TEST_POSITION = false;
static constexpr float GAME_CENTER_X = 4 - 2 + 0.2f, GAME_CENTER_Y = 6 - 1.5f + 0.2f;
static constexpr float GAME_HOME_X = 0, GAME_HOME_Y = 0;
static constexpr float TEST_CENTER_X = 1, TEST_CENTER_Y = 0;
static constexpr float TEST_HOME_X = 0, TEST_HOME_Y = 0;
static constexpr int GO_HOME_HP = 120, GO_CENTER_HP = 400;

enum ChassisState { GO_CENTER,
                    GO_HOME };

// 跨周期状态
static ChassisState chassis_state = GO_CENTER;
static Timer loop_timer;
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
    chassis.set_enable(sentry_enable());
    chassis.set_mode(rc_fn() ? ChassisSteer::FOLLOW_MODE : ChassisSteer::DETACH_MODE);

    // 键盘：WASD 平移，Shift、Ctrl 旋转；断联时直接清零
    if (vt13.is_connect) {
        key_vx = key_speed(key_vx, vt13.key.w, vt13.key.s, VXY_MAX, AXY, DXY, dt);
        key_vy = key_speed(key_vy, vt13.key.a, vt13.key.d, VXY_MAX, AXY, DXY, dt);
        key_wr = key_speed(key_wr, vt13.key.shift, vt13.key.ctrl, WR_MAX, AR, DR, dt);
    } else {
        key_vx = key_vy = key_wr = 0;
    }

    // 自主导航速度，按平移速度在最大转速和保留转速间插值小陀螺
    float vx = 0, vy = 0, wr = 0;
    if (sentry_active()) {
        const bool game_position = FORCE_GAME_POSITION || (!FORCE_TEST_POSITION && sentry_in_game());
        if (chassis_state == GO_CENTER) {
            nav.target_position.x = game_position ? GAME_CENTER_X : TEST_CENTER_X;
            nav.target_position.y = game_position ? GAME_CENTER_Y : TEST_CENTER_Y;
            if (referee.robot_status.current_HP <= GO_HOME_HP)
                chassis_state = GO_HOME;
        } else {
            nav.target_position.x = game_position ? GAME_HOME_X : TEST_HOME_X;
            nav.target_position.y = game_position ? GAME_HOME_Y : TEST_HOME_Y;
            if (referee.robot_status.current_HP >= GO_CENTER_HP)
                chassis_state = GO_CENTER;
        }
        std::tie(vx, vy) = rotate(nav.chassis_speed.vx, nav.chassis_speed.vy, yaw2.position);
        const float speed = std::hypotf(vx, vy);
        wr = map(speed, 0.0f, VXY_MAX, WR_MAX, WR_MIN);
    } else {
        chassis_state = GO_CENTER;
        nav.target_position = {};
    }

    // 叠加摇杆与键盘，断联时摇杆已复位为 0
    vx += (vt13.x + fsi6x.x) * VXY_MAX + key_vx;
    vy += (vt13.y + fsi6x.y) * VXY_MAX + key_vy;
    wr += vt13.wheel * WR_MAX + key_wr;
    chassis.set_speed(std::clamp(vx, -VXY_MAX, VXY_MAX), std::clamp(vy, -VXY_MAX, VXY_MAX), std::clamp(wr, -WR_MAX, WR_MAX));

    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power.chassis_ref);
    chassis.update();
}
