#include "led.hpp"
#include "rc.hpp"
#include "motor.hpp"

static constexpr UnitFloat MAX_SPEED = 360 * deg_s;

void dji_can_send() {
    const int16_t cmd5 = motor.GetVoltageCmd();

    uint8_t data[8];
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x1FF, data, 8);
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

    const UnitFloat<rpm> speed = rc.yaw * MAX_SPEED;

    // 速度闭环
    // motor.SetSpeed(speed);

    // 速度-角度闭环
    static BSP::Dwt dwt;
    static Angle<deg> angle;
    const UnitFloat dt = dwt.UpdateDT();
    angle += speed * dt;
    angle = motor.SetAngle(angle, speed);

    motor.OnLoop();
    dji_can_send();
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