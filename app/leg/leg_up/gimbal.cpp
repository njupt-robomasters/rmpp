#include "gimbal.hpp"
#include "app.hpp"
#include "chassis.hpp"

void gimbal_loop() {
    attitude.update();

    gimbal.set_enable(vt13.is_enable);

    // 自瞄：VT13 photo 或鼠标右键，识别到目标时跟随视觉角度
    // if (vt13.is_auto_aim && aim.vision.is_detect)
    //     gimbal.set_angle(aim.vision.yaw, aim.vision.pitch);
    // else
    gimbal.set_speed(vt13.yaw_speed, vt13.pitch_speed);

    // R 键掉头
    // static bool last_r = false;
    // if (vt13.key.r && !last_r) {
    //     gimbal.set_angle(gimbal.yaw.angle_ref + 180 * deg, gimbal.pitch.angle_ref);
    // }
    // last_r = vt13.key.r;

    gimbal.set_chassis_wr(leg_protocol.received.wr);
    gimbal.update(attitude.q, imu.gyro);
}
