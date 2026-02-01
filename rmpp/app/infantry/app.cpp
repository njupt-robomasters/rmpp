#include "led.hpp"
#include "robot.hpp"

void send_can_cmd() {
    // CAN1
    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = w3.GetCanCmd();
    const int16_t cmd4 = w4.GetCanCmd();
    const int16_t cmd5 = yaw.GetCanCmd();
    const int16_t cmd6 = 0;
    const int16_t cmd7 = shoot.GetCanCmd();
    const int16_t cmd8 = 0;

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
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = cmd8 >> 8;
    data[7] = cmd8;
    BSP::CAN::TransmitStd(1, 0x1FF, data);

    ui.SendCanCmd();

    // CAN2
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