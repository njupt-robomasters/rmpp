#include "gimbal.hpp"
#include "app.hpp"
#include "chassis.hpp"

void gimbal_loop() {
    attitude.update();

    gimbal.set_enable(vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S));

    // 摇杆：射击开关打开时 pitch 摇杆改为调节弹频，不再控制云台
    const float yaw_speed = YAW_SPEED_MAX * (vt13.yaw + vt13.mouse.yaw);
    const float pitch_speed = PITCH_SPEED_MAX * ((vt13.trigger ? 0.0f : -vt13.pitch) - vt13.mouse.pitch);

    // 自瞄：VT13 photo，识别到目标时跟随视觉角度
    // const bool is_auto_aim = vt13.photo || vt13.mouse.right;
    // if (is_auto_aim && aim.vision.is_detect)
    //     gimbal.set_angle(aim.vision.yaw, aim.vision.pitch);
    // else
    gimbal.set_speed(yaw_speed, pitch_speed);

    // R 键掉头
    // static bool last_r = false;
    // if (vt13.key.r && !last_r) {
    //     gimbal.set_angle(gimbal.yaw.angle_ref + 180 * deg, gimbal.pitch.angle_ref);
    // }
    // last_r = vt13.key.r;

    gimbal.set_chassis_wr(down.wr);
    gimbal.update(attitude.q, imu.gyro);
}
