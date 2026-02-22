#include "../led.hpp"
#include "../rc.hpp"
#include "../gimbal.hpp"

static constexpr UnitFloat YAW_MAX_SPEED = 360 * deg_s;
static constexpr UnitFloat PITCH_MAX_SPEED = 360 * deg_s;

void send_can_cmd() {
    const int16_t cmd5 = yaw.GetCanCmd();
    uint8_t data[8];
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x1FF, data);

    pitch.SendCanCmd();
}

void setup() {
    BSP::Init();

    gimbal.SetMode(Gimbal::IMU_MODE);;
}

void loop() {
    led.OnLoop();

    rc.OnLoop();

    gimbal.SetEnable(rc.is_enable);
    const UnitFloat<deg_s> yaw_speed = rc.yaw * YAW_MAX_SPEED;
    const UnitFloat<deg_s> pitch_speed = rc.pitch * PITCH_MAX_SPEED;
    gimbal.SetSpeed(yaw_speed, pitch_speed);
    imu.OnLoop();
    gimbal.OnLoop();

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