#include "led.hpp"
#include "rc.hpp"
#include "shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 5 * Hz;

void send_can_cmd() {
    rub_left.SendCanCmd();
    rub_right.SendCanCmd();

    const int16_t cmd7 = shoot.GetCanCmd();
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x1FF, data, 8);
}

void setup() {
    BSP::Init();

    shooter.SetBulletSpeed(22 * m_s);
}

void loop() {
    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        shooter.SetEnable(false);
    } else {
        shooter.SetEnable(true);
    }

    if (rc.mode == VT13::S) {
        shooter.SetBulletFreq(rc.x * BULLET_FREQ);
        shooter.SetRub(true);
        shooter.SetShoot(true);
    } else {
        shooter.SetRub(false);
        shooter.SetShoot(false);
    }

    shooter.OnLoop();

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