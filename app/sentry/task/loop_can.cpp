#include "app.hpp"

void loop_can() {
    // 底盘
    const int16_t cmd7 = chassis.m_servo1.GetVoltageCmd();
    const int16_t cmd8 = chassis.m_servo2.GetVoltageCmd();
    const int16_t cmd1 = chassis.m_wheel1.GetCurrentCmd();
    const int16_t cmd2 = chassis.m_wheel2.GetCurrentCmd();

    // 云台
    const int16_t cmd5 = gimbal.m_yaw2.GetVoltageCmd();

    // 发射机构
    const int16_t cmd6 = shooter.m_shoot.GetCurrentCmd();

    uint8_t data[8];

    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
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
}