#include "led.hpp"
#include "rc.hpp"
#include "motor.hpp"

static constexpr UnitFloat BULLET_SPEED = 12 * m_s;
static constexpr UnitFloat<rpm> RUB_SPEED = BULLET_SPEED / (3 * cm);

void setup() {
    BSP::Init();
}

void loop() {
    static uint32_t send_cnt;

    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        rub_left1.SetEnable(false);
        rub_right1.SetEnable(false);
        rub_left2.SetEnable(false);
        rub_right2.SetEnable(false);
    } else {
        rub_left1.SetEnable(true);
        rub_right1.SetEnable(true);
        rub_left2.SetEnable(true);
        rub_right2.SetEnable(true);
    }

    if (rc.swc == FSi6X::MID || rc.swc == FSi6X::DOWN) {
        rub_left1.SetSpeed(RUB_SPEED);
        rub_right1.SetSpeed(RUB_SPEED);
        rub_left2.SetSpeed(RUB_SPEED);
        rub_right2.SetSpeed(RUB_SPEED);
    } else {
        rub_left1.SetSpeed(0 * rpm);
        rub_right1.SetSpeed(0 * rpm);
        rub_left2.SetSpeed(0 * rpm);
        rub_right2.SetSpeed(0 * rpm);
    }

    switch (send_cnt++ % 4) {
        case 0:
            rub_left1.OnLoop();
            break;
        case 1:
            rub_right1.OnLoop();
            break;
        case 2:
            rub_left2.OnLoop();
            break;
        case 3:
            rub_right2.OnLoop();
            break;
    }
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