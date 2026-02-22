#include "../led.hpp"
#include "../rc.hpp"
#include "../shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 1 * Hz;
static constexpr UnitFloat BULLET_PER_REV = 6 * (Hz / rps);
static constexpr UnitFloat<rpm> SHOOT_SPEED = BULLET_FREQ / BULLET_PER_REV;

void send_can_cmd() {
    shoot.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();

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