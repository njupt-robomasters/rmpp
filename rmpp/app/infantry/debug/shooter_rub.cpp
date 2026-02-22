#include "../led.hpp"
#include "../rc.hpp"
#include "../shooter.hpp"

static constexpr UnitFloat BULLET_SPEED = 25 * m_s;
static constexpr UnitFloat<rpm> RUB_SPEED = BULLET_SPEED / (3 * cm);

void send_can_cmd() {
    rub1.SendCanCmd();
    rub2.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();

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