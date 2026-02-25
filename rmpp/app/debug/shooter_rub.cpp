#include "../infantry/misc.hpp"
#include "../infantry/shooter.hpp"

static constexpr UnitFloat BULLET_SPEED = 25 * m_s;
static constexpr UnitFloat<rpm> RUB_SPEED = BULLET_SPEED / (3 * cm);

void send_can_cmd() {
    // 步兵、哨兵
    rub1.SendCanCmd();
    rub2.SendCanCmd();

    // 英雄
    // rub1.SendCanCmd();
    // rub2.SendCanCmd();
    // rub3.SendCanCmd();
    // rub4.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    rc.OnLoop();

    rub1.SetEnable(rc.is_enable);
    rub2.SetEnable(rc.is_enable);

    if (rc.is_enable) {
        rub1.SetSpeed(RUB_SPEED);
        rub2.SetSpeed(RUB_SPEED);
    } else {
        rub1.SetSpeed(0 * rpm);
        rub2.SetSpeed(0 * rpm);
    }

    rub1.OnLoop();
    rub2.OnLoop();

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