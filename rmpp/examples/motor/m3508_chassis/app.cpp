#include "led.hpp"
#include "rc.hpp"
#include "motor.hpp"

static constexpr UnitFloat MAX_SPEED = 60 * rpm;

void send_can_cmd() {
    const int16_t cmd1 = motor.GetCanCmd();
    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(motor.config.can_port, 0x200, data, 8);
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        motor.SetEnable(false);
    } else {
        motor.SetEnable(true);
    }

    const UnitFloat<rpm> speed = rc.x * MAX_SPEED;
    motor.SetSpeed(speed);

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