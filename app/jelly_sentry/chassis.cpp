#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "sentry.hpp"

void chassis_loop() {
    chassis.set_enable(sentry_enable());
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure); // 前进正方向为云台方向

    if (sentry_active()) {
        sentry_chassis();
    } else {
        // VT13：摇杆叠加键盘斜坡，速度为云台系，断联时为 0
        chassis.set_mode(ChassisMatrix::DETACH_MODE);
        chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);
    }

    // 超级电容
    const bool is_cap_enable = vt13.is_enable || sentry_in_game();
    super_cap.set_enable(is_cap_enable);
    super_cap.set_boost_enable(is_cap_enable);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();

    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
