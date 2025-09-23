#include "app.hpp"

extern "C" void task_can_entry(const void* argument) {
    while (true) {
        // 底盘
        const int16_t cmd7 = chassis.m_servo1.GetCurrentCMD(); // ID：7
        const int16_t cmd8 = chassis.m_servo2.GetCurrentCMD(); // ID：8
        const int16_t cmd1 = chassis.m_wheel1.GetCurrentCMD(); // ID：1
        const int16_t cmd2 = chassis.m_wheel2.GetCurrentCMD(); // ID：2

        // 云台
        const int16_t cmd5 = gimbal.m_yaw2.GetCurrentCMD(); // ID：5

        // 发射机构
        const int16_t cmd6 = shooter.m_shoot.GetCurrentCMD(); // ID：6

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

        BSP::OS::Delay(1);
    }
}
