#include "led.hpp"
#include "rc.hpp"
#include "chassis.hpp"

static constexpr UnitFloat VXY_MAX = 2 * m_s; // 极限3.2m/s
static constexpr UnitFloat WR_MAX = 60 * rpm; // 极限155rpm

void dji_can_send() {
    const int16_t cmd1 = w1.GetCurrentCmd();
    const int16_t cmd2 = w2.GetCurrentCmd();
    const int16_t cmd7 = s1.GetVoltageCmd();
    const int16_t cmd8 = s2.GetVoltageCmd();

    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x200, data, 8);

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = cmd8 >> 8;
    data[7] = cmd8;
    BSP::CAN::TransmitStd(1, 0x1FF, data, 8);
}

void setup() {
    BSP::Init();

    chassis.SetPowerLimit(0 * W);
    chassis.SetMode(Chassis::DETACH_MODE);
}

void loop() {
    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        chassis.SetEnable(false);
    } else {
        chassis.SetEnable(true);
    }

    const UnitFloat vx = rc.x * VXY_MAX;
    const UnitFloat vy = rc.y * VXY_MAX;
    const UnitFloat wr = rc.wheel * WR_MAX;
    chassis.SetSpeed(vx, vy, wr);

    chassis.OnLoop();
    dji_can_send();
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