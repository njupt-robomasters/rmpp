#include "led.hpp"
#include "rc.hpp"
#include "chassis.hpp"

static constexpr UnitFloat VXY_MAX = 4 * m_s;  // 极限4.5m/s
static constexpr UnitFloat WR_MAX = 120 * rpm; // 极限130rpm

void send_can_cmd() {
    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = w3.GetCanCmd();
    const int16_t cmd4 = w4.GetCanCmd();

    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;
    BSP::CAN::TransmitStd(2, 0x200, data, 8);
}

void setup() {
    BSP::Init();

    chassis.SetMode(Chassis::DETACH_MODE);
    chassis.SetGimbalYaw(0 * deg);
    chassis.SetPowerLimit(120 * W);
    chassis.SetBufferEnergy(60 * J);
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