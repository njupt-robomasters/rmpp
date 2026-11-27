#include "gimbal.hpp"
#include "app.hpp"
#include "minipc.hpp"

// R 键按下沿掉头
static bool last_r = false;

void gimbal_loop() {
    attitude.update();

    gimbal.set_enable(vt13.is_enable);

    // 云台：R 键掉头；photo 或鼠标右键且识别到目标时跟随视觉角度，否则使用摇杆与鼠标速度
    if (vt13.key.r && !last_r)
        gimbal.turn_yaw(180 * deg);
    last_r = vt13.key.r;

    if (vt13.is_aim && aim.is_detect)
        gimbal.set_angle(aim.yaw, aim.pitch);
    else
        gimbal.set_speed(vt13.yaw_speed, vt13.pitch_speed);

    gimbal.update(attitude.q, imu.gyro);
}
