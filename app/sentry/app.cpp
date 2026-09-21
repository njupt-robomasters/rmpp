#include "bsp.hpp"
#include "math/conversions.hpp"
#include "misc.hpp"
#include "robot.hpp"

using math::ms;

void send_can_cmd() {
    // CAN1
    ui.send_can_cmd();

    const int16_t cmd1 = w1.get_can_cmd();
    const int16_t cmd2 = w2.get_can_cmd();
    const int16_t cmd3 = 0;
    const int16_t cmd4 = 0;
    const int16_t cmd5 = yaw2.get_can_cmd();
    const int16_t cmd6 = shoot.get_can_cmd();
    const int16_t cmd7 = s1.get_can_cmd();
    const int16_t cmd8 = s2.get_can_cmd();

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
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = cmd8 >> 8;
    data[7] = cmd8;
    rmpp::board.can(1).send({.id = 0x1FF, .data = data, .size = 8});

    yaw1.send_can_cmd();

    // CAN2
    pitch.send_can_cmd();
    rub1.send_can_cmd();
    rub2.send_can_cmd();
}

extern "C" void app_main() {
    // imu.calibrate();
    robot.init();

    [[maybe_unused]] static float cpu_usage{};
    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            robot.on_loop();
            send_can_cmd();
            const float running_time = timer.get_dt();
            cpu_usage = running_time / (1 * ms);
        }
    }
}