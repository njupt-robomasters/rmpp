#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "gimbal.hpp"
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
    const int16_t cmd5 = yaw.command_value();
    CanFrame frame{.id = 0x1FF, .size = 8};
    frame.data[0] = cmd5 >> 8;
    frame.data[1] = cmd5;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    Can(1).send(frame);

    pitch.send_can_cmd();
}

extern "C" void app_main() {
    gimbal.set_mode(Gimbal::IMU_MODE);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            rc.update();

            gimbal.set_enable(rc.is_enable);
            const float yaw_speed = clamp(rc.wyaw, rc.config.wyaw);
            const float pitch_speed = clamp(rc.wpitch, rc.config.wpitch);
            gimbal.set_speed(yaw_speed, pitch_speed);
            gimbal.update();

            send_can_cmd();
        }
    }
}
