#include "../infantry/misc.hpp"
#include "../infantry/shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 5 * Hz;
static const UnitFloat<rpm> SHOOT_SPEED = BULLET_FREQ / shooter.config.bullet_per_rev;

void send_can_cmd() {
    // 步兵、哨兵
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
    BSP::CAN::Transmit(shoot.config.can_port, 0x1FF, data, 8);

    // 英雄
    // shoot.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    rc.OnLoop();

    shoot.SetEnable(rc.is_enable);
    const UnitFloat<rpm> speed = rc.x * SHOOT_SPEED;
    shoot.SetSpeed(speed);
    shoot.OnLoop();

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