#include "led.hpp"
#include "rc.hpp"
#include "shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 5 * Hz;

void send_can_cmd() {
    static uint32_t send_cnt;
    switch (send_cnt++ % 4) {
        case 0:
            rub_left1.SendCanCmd();
            break;
        case 1:
            rub_right1.SendCanCmd();
            break;
        case 2:
            rub_left2.SendCanCmd();
            break;
        case 3:
            rub_right2.SendCanCmd();
            break;
        case 4:
            break;
    }

    shoot.SendCanCmd();
}

void setup() {
    BSP::Init();

    shooter.SetBulletSpeed(12 * m_s);
}

void loop() {
    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        shooter.SetEnable(false);
    } else {
        shooter.SetEnable(true);
    }

    if (rc.swc == FSi6X::MID || rc.swc == FSi6X::DOWN) {
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