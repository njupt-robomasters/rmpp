#include "../led.hpp"
#include "../rc.hpp"
#include "../gimbal.hpp"

static constexpr UnitFloat MAX_SPEED = 360 * deg_s;

void send_can_cmd() {
    pitch.SendCanCmd();
}

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();

    rc.OnLoop();

    pitch.SetEnable(rc.is_enable);
    const UnitFloat<rpm> speed = rc.pitch * MAX_SPEED;

    // 速度闭环
    // pitch.SetSpeed(speed);

    // 角度闭环
    static BSP::Dwt dwt;
    static Angle<deg> angle;
    const UnitFloat dt = dwt.UpdateDT();
    angle += speed * dt;
    angle = pitch.SetAngle(angle, speed);

    pitch.OnLoop();
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