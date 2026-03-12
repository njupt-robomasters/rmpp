#include "motor/CustomMotor.hpp"

CustomMotor motor({
    .can_port = 1,
    .master_id = 0x1FF,
    .slave_id = 0x205,
    .control_mode = Motor::OPEN_LOOP_MODE,
});

void send_can_cmd() {
    const int16_t cmd5 = motor.GetCanCmd();
    uint8_t data[8];
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::Transmit(motor.config.can_port, 0x1FF, data, 8);
}

void setup() {
    BSP::Init();
}

void loop() {
    motor.SetCurrent(0.1 * A);
    motor.OnLoop();
    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
        }
    }
}