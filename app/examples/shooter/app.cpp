#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "rc/rc.hpp"
#include "shooter.hpp"
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
#include "math/unit.hpp"

void send_can_cmd() {
    const int16_t cmd6 = shoot.command_value();
    CanFrame frame{.id = 0x1FF, .size = 8};
    frame.data[0] = 0;
    frame.data[1] = 0;
    frame.data[2] = cmd6 >> 8;
    frame.data[3] = cmd6;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    Can(1).send(frame);
    rub1.send_can_cmd();
    rub2.send_can_cmd();
}

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            rc.update();

            shooter.set_enable(rc.is_enable);
            const auto command = rc.shoot_command(false);
            shooter.set_rub(rc.is_rub);
            shooter.set_shoot(command.is_shoot, command.rate);
            shooter.update();

            send_can_cmd();
        }
    }
}
