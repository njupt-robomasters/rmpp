#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "chassis.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include <algorithm>

static constexpr float VXY_MAX = 1.0f;    // 平移最大速度，m/s
static constexpr float WR_MAX = 60 * rpm; // 旋转最大角速度，rad/s

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

void send_can_cmd() {
    CanFrame frame{.id = 0x200, .size = 8};
    w1.write_can_frame(frame);
    w2.write_can_frame(frame);
    w3.write_can_frame(frame);
    w4.write_can_frame(frame);
    Can(1).send(frame);
}

extern "C" void app_main() {
    chassis.set_mode(ChassisMatrix::DETACH_MODE);
    chassis.set_gimbal_yaw(0 * deg);
    super_cap.set_referee_limit(120);
    super_cap.set_referee_energy(60);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            vt13.update();
            fsi6x.update();

            chassis.set_enable(rc_enable());
            const float vx = std::clamp((vt13.x + fsi6x.x) * VXY_MAX, -VXY_MAX, VXY_MAX);
            const float vy = std::clamp((vt13.y + fsi6x.y) * VXY_MAX, -VXY_MAX, VXY_MAX);
            const float wr = vt13.wheel * WR_MAX;
            chassis.set_speed(vx, vy, wr);
            chassis.update();

            send_can_cmd();
        }
    }
}
