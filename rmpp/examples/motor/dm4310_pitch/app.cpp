#include "led.hpp"
#include "rc.hpp"
#include "motor.hpp"

static constexpr UnitFloat MAX_SPEED = 360 * deg_s;

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

    const UnitFloat<rpm> speed = rc.pitch * MAX_SPEED;

    // 速度闭环
    motor.SetSpeed(speed);

    // 速度-角度闭环
    // static BSP::Dwt dwt;
    // static Angle<deg> angle;
    // const UnitFloat dt = dwt.UpdateDT();
    // angle += speed * dt;
    // angle = motor.SetAngle(angle, speed);

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