#include "../led.hpp"
#include "../rc.hpp"
#include "../gimbal.hpp"

static constexpr UnitFloat MAX_SPEED = 360 * deg_s;

void send_can_cmd() {
    yaw1.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();

    rc.OnLoop();

    yaw1.SetEnable(rc.is_enable);
    const UnitFloat<rpm> speed = rc.yaw * MAX_SPEED;

    // 速度闭环
    // motor.SetSpeed(speed);

    // 角度闭环
    static BSP::Dwt dwt;
    static Angle<deg> angle;
    const UnitFloat dt = dwt.UpdateDT();
    angle += speed * dt;
    angle = yaw1.SetAngle(angle, speed);

    yaw1.OnLoop();

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