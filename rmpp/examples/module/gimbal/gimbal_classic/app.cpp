#include "led.hpp"
#include "rc.hpp"
#include "gimbal.hpp"

static constexpr UnitFloat YAW_MAX_SPEED = 360 * deg_s;
static constexpr UnitFloat PITCH_MAX_SPEED = 360 * deg_s;

void send_can_cmd() {
    pitch.SendCanCmd();

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
}

void setup() {
    BSP::Init();

    gimbal.SetMode(Gimbal::IMU_MODE);;
}

void loop() {
    led.OnLoop();
    rc.OnLoop();
    imu.OnLoop();

    if (rc.is_connect == false || rc.is_protect == true) {
        gimbal.SetEnable(false);
    } else {
        gimbal.SetEnable(true);
    }

    const UnitFloat yaw_speed = rc.yaw * YAW_MAX_SPEED;
    const UnitFloat pitch_speed = rc.pitch * PITCH_MAX_SPEED;
    gimbal.SetSpeed(yaw_speed, pitch_speed);

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