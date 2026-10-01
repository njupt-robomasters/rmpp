#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "motor/dji/c620.hpp"

static C620 motor({
    .bus = {.can = Can(1), .rx_id = 0x201},
    .control = {.mode = Motor::OPEN_LOOP_MODE},
});

static void send_can_cmd() {
    CanFrame frame{.id = 0x200, .size = 8};
    motor.write_can_frame(frame);
    Can(1).send(frame);
}

extern "C" void app_main() {

    motor.set_enable(true);
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            motor.set_current(0.1f);
            motor.update();
            send_can_cmd();
        }
    }
}
