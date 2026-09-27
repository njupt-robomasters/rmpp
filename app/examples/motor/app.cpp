#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "motor/dji/m3508.hpp"

static M3508 motor({
    .protocol = {.can = Can(1), .master_id = 0x201, .slave_id = 0x200},
    .control = {.control_mode = Motor::OPEN_LOOP_MODE},
});

static bool send_can_cmd() {
    const int16_t command = motor.command_value();
    CanFrame frame{.id = 0x200, .size = 8};
    frame.data[0] = command >> 8;
    frame.data[1] = command;
    return motor.config.protocol.can.send(frame);
}

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            motor.set_current(0.1f);
            motor.update_control();
            send_can_cmd();
        }
    }
}
