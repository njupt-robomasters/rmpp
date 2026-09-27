#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "chassis.hpp"
#include "math/math.hpp"
#include "rc/rc.hpp"

static FSi6X fsi6x({.uart = Uart(3)});
static VT13 vt13({.uart = Uart(6)});
static Rc rc({
                 .vxy_max = 1.0f,
                 .wr_max = 60 * rpm,
                 .axy = 100.0f,
                 .dxy = 100.0f,
                 .ar = 1000 * rpm,
                 .dr = 1000 * rpm,
                 .wyaw = 360 * deg_s,
                 .wpitch = 360 * deg_s,
                 .ayaw = 3600 * deg_s,
                 .apitch = 3600 * deg_s,
             },
             fsi6x, vt13);

void send_can_cmd() {
    const int16_t cmd1 = w1.command_value();
    const int16_t cmd2 = w2.command_value();
    const int16_t cmd3 = w3.command_value();
    const int16_t cmd4 = w4.command_value();
    CanFrame frame{.id = 0x200, .size = 8};
    frame.data[0] = cmd1 >> 8;
    frame.data[1] = cmd1;
    frame.data[2] = cmd2 >> 8;
    frame.data[3] = cmd2;
    frame.data[4] = cmd3 >> 8;
    frame.data[5] = cmd3;
    frame.data[6] = cmd4 >> 8;
    frame.data[7] = cmd4;
    Can(1).send(frame);
}

extern "C" void app_main() {
    chassis.set_mode(Chassis::DETACH_MODE);
    chassis.set_gimbal_yaw(0 * deg);
    chassis.set_power_limit(120);
    chassis.set_buffer_energy(60);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            rc.update();

            chassis.set_enable(rc.is_enable);
            const float vx = clamp(rc.vx, rc.config.vxy_max);
            const float vy = clamp(rc.vy, rc.config.vxy_max);
            const float wr = clamp(rc.wr, rc.config.wr_max);
            chassis.set_speed(vx, vy, wr);
            chassis.update();

            send_can_cmd();
        }
    }
}
