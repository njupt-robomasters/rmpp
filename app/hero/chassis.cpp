#include "app.hpp"
#include "math/math.hpp"

void chassis_loop() {
    chassis.set_enable(rc.is_enable);
    if (rc.is_fn)
        chassis.set_mode(Chassis::FOLLOW_MODE);
    chassis.set_speed(clamp(rc.vx, rc.config.vxy_max), clamp(rc.vy, rc.config.vxy_max), clamp(rc.wr, rc.config.wr_max));
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);
    chassis.set_power_limit(referee.chassis.power_limit);
    chassis.set_buffer_energy(referee.chassis.buffer_energy);
    chassis.update();
}
