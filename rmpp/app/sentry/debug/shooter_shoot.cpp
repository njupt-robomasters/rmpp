#include "../led.hpp"
#include "../rc.hpp"
#include "../shooter.hpp"

static constexpr UnitFloat BULLET_FREQ = 5 * Hz;
static constexpr UnitFloat BULLET_PER_REV = 9.0f * (41.0f / 50.0f) * (Hz / rps);
static constexpr UnitFloat<rpm> SHOOT_SPEED = BULLET_FREQ / BULLET_PER_REV;

void send_can_cmd() {
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
    BSP::CAN::TransmitStd(shoot.config.can_port, 0x1FF, data, 8);
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