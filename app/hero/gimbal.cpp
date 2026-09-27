#include "app.hpp"
#include "math/math.hpp"

void gimbal_loop() {
    gimbal.set_enable(rc.is_enable);
    if (rc.is_calib) {
        gimbal.set_yaw_zero();
        gimbal.save_yaw_offset(storage);
        buzzer.play(Buzzer::G5G5G5);
    }
    if (rc.is_turn_back) {
        gimbal.yaw.ecd.ref += 180 * deg;
        gimbal.yaw.imu.ref += 180 * deg;
    }
    if (rc.is_auto_aim && mavlink.vision.is_detect)
        gimbal.set_angle(mavlink.vision.yaw, mavlink.vision.pitch);
    else
        gimbal.set_speed(clamp(rc.wyaw, rc.config.wyaw), clamp(rc.wpitch, rc.config.wpitch));
    gimbal.set_chassis_wr(chassis.wr.measure);
    gimbal.update();
}
