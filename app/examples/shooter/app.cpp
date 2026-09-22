#include "bsp.hpp"
#include "rc/rc.hpp"
#include "shooter.hpp"
Rc rc({.uart = bsp.uart(3)}, {.uart = bsp.uart(6)});
#include "math/unit.hpp"

static constexpr float BULLET_FREQ = 20;
static constexpr float BULLET_SPEED = 25.0f;

void send_can_cmd() {
    const int16_t cmd6 = shoot.get_can_cmd();
    std::array<uint8_t, 8> data{};
    data[0] = 0;
    data[1] = 0;
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    bsp.can(1).send({.id = 0x1FF, .data = data, .size = 8});
    rub1.send_can_cmd();
    rub2.send_can_cmd();
}

extern "C" void app_main() {
    shooter.set_bullet_speed(BULLET_SPEED);

    Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            rc.on_loop();

            shooter.set_enable(rc.is_enable);
            shooter.set_bullet_freq(rc.x * BULLET_FREQ);
            shooter.set_rub(rc.is_rub);
            shooter.set_shoot(true);
            shooter.on_loop();

            send_can_cmd();
        }
    }
}
