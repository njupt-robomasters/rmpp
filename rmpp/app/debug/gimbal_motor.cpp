#include "../infantry/gimbal.hpp"
#include "../infantry/misc.hpp"
#include "math/Angle.hpp"
#include "math/Conversions.hpp"

using math::Angle;
using math::deg_s;
using math::ms;

static constexpr float MAX_SPEED = 360 * deg_s;

void send_can_cmd() {
    const int16_t cmd5 = motor.GetCanCmd();
    uint8_t data[8];
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::Transmit(motor.config.can_port, 0x1FF, data, 8);

    // motor.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    rc.OnLoop();

    motor.SetEnable(rc.is_enable);
    const float speed = rc.x * MAX_SPEED;

    // 速度闭环
    // motor.config.control_mode = Motor::SPEED_MODE;
    // motor.SetSpeed(speed);

    // 角度闭环
    static BSP::Dwt dwt;
    static Angle angle;
    const float dt = dwt.UpdateDT();
    angle += speed * dt;
    motor.config.control_mode = Motor::ANGLE_SPEED_MODE;
    angle = motor.SetAngle(angle, speed);

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