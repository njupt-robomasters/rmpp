#include "led.hpp"
#include "robot.hpp"

void send_can_cmd() {
    // 底盘
    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = w3.GetCanCmd();
    const int16_t cmd4 = w4.GetCanCmd();
    // 云台
    const int16_t cmd5 = yaw.GetCanCmd();
    // 发射机构
    const int16_t cmd7 = shoot.GetCanCmd();

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

    pitch.SendCanCmd();
    rub_left.SendCanCmd();
    rub_right.SendCanCmd();
}

void setup() {
    BSP::Init();
    // imu.Calibrate();
}

void loop() {
    led.OnLoop();
    robot.OnLoop();
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