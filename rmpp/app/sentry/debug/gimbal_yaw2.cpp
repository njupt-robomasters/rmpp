#include "../led.hpp"
#include "../rc.hpp"
#include "../gimbal.hpp"

static constexpr UnitFloat MAX_SPEED = 360 * deg_s;

void can_send_cmd() {
    const int16_t cmd5 = yaw2.GetCanCmd();
    uint8_t data[8];
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(yaw2.config.can_port, 0x1FF, data, 8);
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();

    rc.OnLoop();

    yaw2.SetEnable(rc.is_enable);
    const UnitFloat<rpm> speed = rc.yaw * MAX_SPEED;

    // 速度闭环
    // yaw2.SetSpeed(speed);

    // 角度闭环
    static BSP::Dwt dwt;
    static Angle<deg> angle;
    const UnitFloat dt = dwt.UpdateDT();
    angle += speed * dt;
    angle = yaw2.SetAngle(angle, speed);

    yaw2.OnLoop();

    can_send_cmd();
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