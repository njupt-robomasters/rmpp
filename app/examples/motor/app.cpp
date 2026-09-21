#include "bsp.hpp"
#include "math/conversions.hpp"
#include "motor/custom_motor.hpp"

using math::ms;

CustomMotor motor({
    .can_port = 1,
    .master_id = 0x1FF,
    .slave_id = 0x205,
    .control_mode = Motor::OPEN_LOOP_MODE,
});

void send_can_cmd() {
    const int16_t cmd5 = motor.get_can_cmd();
    std::array<uint8_t, 8> data{};
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    rmpp::board.can(motor.config.can_port).send({.id = 0x1FF, .data = data, .size = 8});
}

extern "C" void app_main() {

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            motor.set_current(0.1);
            motor.on_loop();
            send_can_cmd();
        }
    }
}
