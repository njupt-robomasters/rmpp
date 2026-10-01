#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "math/unit.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include "shooter.hpp"

static constexpr float BULLET_SPEED = 25.0f; // 弹速，m/s
static constexpr float BULLET_FREQ = 20.0f;  // 弹频，发/秒

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

void send_can_cmd() {
    CanFrame frame{.id = 0x1FF, .size = 8};
    shoot.write_can_frame(frame);
    Can(1).send(frame);
    rub1.send();
    rub2.send();
}

extern "C" void app_main() {
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            vt13.update();
            fsi6x.update();

            // VT13 S 挡或 FSi6X SWC 下开摩擦轮，VT13 trigger 或 FSi6X SWB 下射击
            shooter.set_enable(rc_enable());
            shooter.set_rub(vt13.mode == VT13::S || fsi6x.swc == FSi6X::DOWN);
            shooter.set_shoot(vt13.trigger || fsi6x.swb == FSi6X::DOWN);
            shooter.set_bullet_speed(BULLET_SPEED);
            shooter.set_bullet_freq(BULLET_FREQ);
            shooter.update();

            send_can_cmd();
        }
    }
}
