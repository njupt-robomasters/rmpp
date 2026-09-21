#include "bsp.hpp"
#include "chassis.hpp"
#include "rc/rc.hpp"
Rc rc({}, {});
#include "math/conversions.hpp"

using math::deg;
using math::ms;
using math::rpm;

static constexpr float VXY_MAX = 1.0f;
static constexpr float WR_MAX = 60 * rpm;

void send_can_cmd() {
    const int16_t cmd1 = w1.get_can_cmd();
    const int16_t cmd2 = w2.get_can_cmd();
    const int16_t cmd3 = w3.get_can_cmd();
    const int16_t cmd4 = w4.get_can_cmd();
    std::array<uint8_t, 8> data{};
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;
    rmpp::board.can(1).send({.id = 0x200, .data = data, .size = 8});
}

extern "C" void app_main() {
    chassis.set_mode(Chassis::DETACH_MODE);
    chassis.set_gimbal_yaw(0 * deg);
    chassis.set_power_limit(120);
    chassis.set_buffer_energy(60);

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            rc.on_loop();

            chassis.set_enable(rc.is_enable);
            const float vx = rc.x * VXY_MAX;
            const float vy = rc.y * VXY_MAX;
            const float wr = rc.r * WR_MAX;
            chassis.set_speed(vx, vy, wr);
            chassis.on_loop();

            send_can_cmd();
        }
    }
}
