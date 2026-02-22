#include "../led.hpp"
#include "../rc.hpp"
#include "../shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 5 * Hz;
static constexpr UnitFloat BULLET_SPEED = 12 * m_s;

void send_can_cmd() {
    shoot.SendCanCmd();

    rub1.SendCanCmd();
    rub2.SendCanCmd();
    rub3.SendCanCmd();
    rub4.SendCanCmd();
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