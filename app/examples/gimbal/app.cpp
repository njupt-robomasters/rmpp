#include "bsp.hpp"
#include "gimbal.hpp"
#include "rc/rc.hpp"
Rc rc({}, {});
#include "math/conversions.hpp"

using math::deg_s;
using math::ms;

static constexpr float YAW_SPEED_MAX = 360 * deg_s;
static constexpr float PITCH_SPEED_MAX = 360 * deg_s;

void send_can_cmd() {
    const int16_t cmd5 = yaw.get_can_cmd();
    std::array<uint8_t, 8> data{};
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    rmpp::board.can(1).send({.id = 0x1FF, .data = data, .size = 8});
    pitch.send_can_cmd();
}

extern "C" void app_main() {
    gimbal.set_mode(Gimbal::IMU_MODE);

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            rc.on_loop();

            gimbal.set_enable(rc.is_enable);
            const float yaw_speed = rc.yaw * YAW_SPEED_MAX;
            const float pitch_speed = rc.pitch * PITCH_SPEED_MAX;
            gimbal.set_speed(yaw_speed, pitch_speed);
            imu.on_loop();
            gimbal.on_loop();

            send_can_cmd();
        }
    }
}
