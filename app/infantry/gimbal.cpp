#include "app.hpp"
#include "bsp/timer.hpp"
#include "melody/music/g5g5g5.hpp"
#include <algorithm>

// 操控参数
static constexpr float WYAW_MAX = 360 * deg_s;   // yaw 最大角速度，rad/s
static constexpr float WPITCH_MAX = 360 * deg_s; // pitch 最大角速度，rad/s
static constexpr float AYAW = 3600 * deg_s;      // 鼠标 yaw 最大角加速度，rad/s^2
static constexpr float APITCH = 3600 * deg_s;    // 鼠标 pitch 最大角加速度，rad/s^2

// 跨周期状态
static Timer loop_timer;
static float mouse_wyaw = 0, mouse_wpitch = 0;
static bool last_calib = false, last_r = false;

void gimbal_loop() {
    const float dt = loop_timer.restart();
    gimbal.set_enable(rc_enable());

    // yaw 零点校准：VT13 C 挡按住 fn 和 photo，或 FSi6X SWC 上、SWA 下、SWD 下，按下时触发一次
    const bool calib = (vt13.mode == VT13::C && vt13.fn && vt13.photo) ||
                       (fsi6x.swc == FSi6X::UP && fsi6x.swa == FSi6X::DOWN && fsi6x.swd == FSi6X::DOWN);
    if (calib && !last_calib) {
        gimbal.set_yaw_zero();
        gimbal.save_yaw_offset(kv_store);
        melody.play(G5G5G5);
    }
    last_calib = calib;

    // R 键掉头
    if (vt13.key.r && !last_r) {
        gimbal.yaw.ecd.ref += 180 * deg;
        gimbal.yaw.imu.ref += 180 * deg;
    }
    last_r = vt13.key.r;

    // 鼠标限制角加速度；断联时直接清零
    if (vt13.is_connect) {
        mouse_wyaw += std::clamp(vt13.mouse.yaw * WYAW_MAX - mouse_wyaw, -AYAW * dt, AYAW * dt);
        mouse_wpitch += std::clamp(vt13.mouse.pitch * WPITCH_MAX - mouse_wpitch, -APITCH * dt, APITCH * dt);
    } else {
        mouse_wyaw = mouse_wpitch = 0;
    }

    // 摇杆：射击开关打开时 pitch 摇杆改为调节弹频，不再控制云台
    const float yaw = vt13.yaw + fsi6x.yaw;
    float pitch = 0;
    if (!vt13.trigger)
        pitch += vt13.pitch;
    if (fsi6x.swb != FSi6X::DOWN)
        pitch += fsi6x.pitch;
    const float wyaw = yaw * WYAW_MAX + mouse_wyaw;
    const float wpitch = pitch * WPITCH_MAX + mouse_wpitch;

    // 自瞄：VT13 photo、FSi6X SWD 下或按住鼠标右键，识别到目标时跟随视觉角度
    const bool is_auto_aim = vt13.photo || fsi6x.swd == FSi6X::DOWN || vt13.mouse.right;
    if (is_auto_aim && aim.vision.is_detect)
        gimbal.set_angle(aim.vision.yaw, aim.vision.pitch);
    else
        gimbal.set_speed(std::clamp(wyaw, -WYAW_MAX, WYAW_MAX), std::clamp(wpitch, -WPITCH_MAX, WPITCH_MAX));

    gimbal.set_chassis_wr(chassis.wr.measure);
    attitude.update();
    gimbal.update(attitude.yaw, attitude.pitch);
}
