#include "app.hpp"
#include "gimbal.hpp"

// 操控参数
static constexpr float WYAW_MAX = 360 * deg_s;   // yaw 最大角速度，rad/s
static constexpr float WPITCH_MAX = 360 * deg_s; // pitch 最大角速度，rad/s

void gimbal_loop() {
    gimbal.set_enable(vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S));

    // R 键掉头
    // static bool last_r = false;
    // if (vt13.key.r && !last_r) {
    //     gimbal.yaw.ecd.ref += 180 * deg;
    //     gimbal.yaw.imu.ref += 180 * deg;
    // }
    // last_r = vt13.key.r;

    // 摇杆：射击开关打开时 pitch 摇杆改为调节弹频，不再控制云台
    const float wyaw = WYAW_MAX * (vt13.yaw + vt13.mouse.yaw);
    const float wpitch = WPITCH_MAX * ((vt13.trigger ? 0.0f : vt13.pitch) + vt13.mouse.pitch);

    // 自瞄：VT13 photo，识别到目标时跟随视觉角度
    const bool is_auto_aim = vt13.photo || vt13.mouse.right;
    // if (is_auto_aim && aim.vision.is_detect)
    //     gimbal.set_angle(aim.vision.yaw, aim.vision.pitch);
    // else
        gimbal.set_speed(std::clamp(wyaw, -WYAW_MAX, WYAW_MAX), std::clamp(wpitch, -WPITCH_MAX, WPITCH_MAX));

    // gimbal.set_chassis_wr(chassis.wr.measure);
    attitude.update();
    gimbal.update(attitude.yaw, attitude.pitch);
}
