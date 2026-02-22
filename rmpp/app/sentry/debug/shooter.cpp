#include "../led.hpp"
#include "../rc.hpp"
#include "../shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 5 * Hz;
static constexpr UnitFloat BULLET_SPEED = 25 * m_s;

void send_can_cmd() {
    const int16_t cmd6 = shoot.GetCanCmd();
    uint8_t data[8];
    data[0] = 0;
    data[1] = 0;
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x1FF, data, 8);

    rub1.SendCanCmd();
    rub2.SendCanCmd();
}

void setup() {
    BSP::Init();

    shooter.SetBulletSpeed(BULLET_SPEED);
}

void loop() {
    led.OnLoop();

    rc.OnLoop();

    shooter.SetEnable(rc.is_enable);
    shooter.SetBulletFreq(rc.x * BULLET_FREQ);
    shooter.SetRub(rc.is_rub);
    shooter.SetShoot(rc.is_shoot);
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