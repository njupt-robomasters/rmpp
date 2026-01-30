#include "led.hpp"
#include "rc.hpp"
#include "motor.hpp"

static constexpr UnitFloat BULLET_SPEED = 25 * m_s;
static constexpr UnitFloat<rpm> RUB_SPEED = BULLET_SPEED / (3 * cm);

void send_can_cmd() {
    rub_left.SendCanCmd();
    rub_right.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        rub_left.SetEnable(false);
        rub_right.SetEnable(false);
    } else {
        rub_left.SetEnable(true);
        rub_right.SetEnable(true);
    }

    if (rc.mode == VT13::S) {
        rub_left.SetSpeed(RUB_SPEED);
        rub_right.SetSpeed(RUB_SPEED);
    } else {
        rub_left.SetSpeed(0 * rpm);
        rub_right.SetSpeed(0 * rpm);
    }

    rub_left.OnLoop();
    rub_right.OnLoop();

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