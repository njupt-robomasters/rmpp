#include <algorithm>
#include "../../jelly_sentry/gimbal.hpp"
#include "bsp/timer.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

// 哨兵整云台测试：IMU 模式，遥控器左手摇杆控制 yaw、pitch 转速；电机与 PID 使用整车 app/jelly_sentry/gimbal.hpp
// 使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡
// 折叠关节 pitch1：调试器写 pitch1_angle（度）后把 set_pitch1 置 true 生效，生效后自动复位
// 与整车一致：大 yaw CAN1，pitch1、pitch2、小 yaw CAN2；7025 应答上限 500 Hz，每 2 ms 发一帧

static constexpr float WYAW_MAX = 360 * deg_s;   // yaw 最大角速度，rad/s
static constexpr float WPITCH_MAX = 360 * deg_s; // pitch 最大角速度，rad/s

volatile bool set_pitch1 = false;  // 置 true 时应用 pitch1_angle
volatile float pitch1_angle = 0;   // 折叠关节目标角，度

static VT13 vt13({.uart = Uart(1)});
static FSi6X fsi6x({.uart = Uart(5)});

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

static uint8_t yaw2_divider = 0;

static void send_can_cmd() {
    yaw1.send();
    pitch1.send();
    pitch2.send();
    if (++yaw2_divider >= 2) {
        yaw2_divider = 0;
        yaw2.send();
    }
}

extern "C" void app_main() {
    gimbal.set_mode(Gimbal::IMU_MODE);

    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        loop_timer.restart();

        vt13.update();
        fsi6x.update();

        gimbal.set_enable(rc_enable());
        const float yaw_speed = std::clamp((vt13.yaw + fsi6x.yaw) * WYAW_MAX, -WYAW_MAX, WYAW_MAX);
        const float pitch_speed = std::clamp((vt13.pitch + fsi6x.pitch) * WPITCH_MAX, -WPITCH_MAX, WPITCH_MAX);
        gimbal.set_speed(yaw_speed, pitch_speed);
        if (set_pitch1) {
            set_pitch1 = false;
            gimbal.set_pitch1_angle(pitch1_angle * deg);
        }
        gimbal.update();

        send_can_cmd();
    }
}
