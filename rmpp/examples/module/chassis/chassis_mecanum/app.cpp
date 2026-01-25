#include "led.hpp"
#include "rc.hpp"
#include "chassis.hpp"

static constexpr UnitFloat VXY_MAX = 2 * m_s;
static constexpr UnitFloat WR_MAX = 60 * rpm;

void dji_can_send() {
    const int16_t cmd1 = w1.GetCurrentCmd();
    const int16_t cmd2 = w2.GetCurrentCmd();
    const int16_t cmd3 = w3.GetCurrentCmd();
    const int16_t cmd4 = w4.GetCurrentCmd();

    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;
    BSP::CAN::TransmitStd(1, 0x200, data, 8);
}

void setup() {
    BSP::Init();

    chassis.SetPowerLimit(120 * W);
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
    const UnitFloat wr = rc.yaw * WR_MAX;
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