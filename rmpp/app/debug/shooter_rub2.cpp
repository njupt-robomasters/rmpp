#include "../infantry/misc.hpp"
#include "../infantry/shooter.hpp"
#include "math/Conversions.hpp"

using math::cm;
using math::ms;
using math::rpm;

static constexpr float BULLET_SPEED = 25.0f;
static constexpr float RUB_SPEED = BULLET_SPEED / (3 * cm);

void send_can_cmd() {
    rub1.SendCanCmd();
    rub2.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    rc.OnLoop();

    rub1.SetEnable(rc.is_enable);
    rub2.SetEnable(rc.is_enable);

    if (rc.is_rub) {
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