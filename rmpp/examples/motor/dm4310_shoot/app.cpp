#include "led.hpp"
#include "rc.hpp"
#include "motor.hpp"

static constexpr UnitFloat BULLET_FREQ = 2 * Hz;
static constexpr UnitFloat BULLET_PER_REV = 6 * (Hz / rps);
static constexpr UnitFloat<rpm> MAX_SPEED = BULLET_FREQ / BULLET_PER_REV;

void send_can_cmd() {
    motor.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();
    rc.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        motor.SetEnable(false);
    } else {
        motor.SetEnable(true);
    }

    const UnitFloat<rpm> speed = rc.x * MAX_SPEED;
    motor.SetSpeed(speed);

    motor.OnLoop();

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