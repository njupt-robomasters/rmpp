#include "led.hpp"
#include "robot.hpp"

void dji_can_send() {
    // 底盘
    const int16_t cmd1 = w1.GetCurrentCmd();
    const int16_t cmd2 = w2.GetCurrentCmd();
    const int16_t cmd3 = w3.GetCurrentCmd();
    const int16_t cmd4 = w4.GetCurrentCmd();
    // 云台
    const int16_t cmd5 = yaw.GetVoltageCmd();
    // 发射机构
    const int16_t cmd7 = shoot.GetCurrentCmd();

    uint8_t data[8];

    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;
    BSP::CAN::TransmitStd(1, 0x200, data);

    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x1FF, data);
}

void setup() {
    BSP::Init();
    // imu.Calibrate();
}

void loop() {
    led.OnLoop();
    robot.OnLoop();
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