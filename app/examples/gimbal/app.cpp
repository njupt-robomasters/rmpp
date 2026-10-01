#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "gimbal.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include <algorithm>

static constexpr float WYAW_MAX = 360 * deg_s;   // yaw 最大角速度，rad/s
static constexpr float WPITCH_MAX = 360 * deg_s; // pitch 最大角速度，rad/s

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

void send_can_cmd() {
    CanFrame frame{.id = 0x1FF, .size = 8};
    yaw.write_can_frame(frame);
    Can(1).send(frame);

    pitch.send();
}

extern "C" void app_main() {
    gimbal.set_mode(Gimbal::IMU_MODE);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            vt13.update();
            fsi6x.update();

            gimbal.set_enable(rc_enable());
            const float yaw_speed = std::clamp((vt13.yaw + fsi6x.yaw) * WYAW_MAX, -WYAW_MAX, WYAW_MAX);
            const float pitch_speed = std::clamp((vt13.pitch + fsi6x.pitch) * WPITCH_MAX, -WPITCH_MAX, WPITCH_MAX);
            gimbal.set_speed(yaw_speed, pitch_speed);
            gimbal.update();

            send_can_cmd();
        }
    }
}
