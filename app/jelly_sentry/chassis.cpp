#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include "sentry.hpp"

void chassis_loop() {
    chassis.set_enable(vt13.is_enable);
    chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_gimbal_yaw(gimbal.yaw.ecd_measure);

    if (is_sentry_active())
        sentry_chassis();
    else
        chassis.set_speed(vt13.vx, vt13.vy, vt13.wr);

    // 超级电容
    super_cap.set_enable(vt13.is_enable);
    super_cap.set_boost_enable(vt13.is_enable);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();

    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
