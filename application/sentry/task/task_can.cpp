#include "app.hpp"

extern "C" [[noreturn]] void task_can_entry(const void* argument) {
    while (true) {
        const int16_t m6020_1_cmd = chassis.m6020_1.GetCurrentCMD(); // ID：3
        const int16_t m6020_2_cmd = chassis.m6020_2.GetCurrentCMD(); // ID：4
        const int16_t m3508_1_cmd = chassis.m3508_1.GetCurrentCMD(); // ID：1
        const int16_t m3508_2_cmd = chassis.m3508_2.GetCurrentCMD(); // ID：2
        const int16_t m2006_cmd = shooter.m2006.GetCurrentCMD(); // ID：6

        uint8_t data[8];

        data[0] = m3508_1_cmd >> 8;
        data[1] = m3508_1_cmd;
        data[2] = m3508_2_cmd >> 8;
        data[3] = m3508_2_cmd;
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        BSP::CAN::TransmitStd(1, 0x200, data);

        data[0] = 0;
        data[1] = 0;
        data[2] = m2006_cmd >> 8;
        data[3] = m2006_cmd;
        data[4] = m6020_1_cmd >> 8;
        data[5] = m6020_1_cmd;
        data[6] = m6020_2_cmd >> 8;
        data[7] = m6020_2_cmd;
        BSP::CAN::TransmitStd(1, 0x1FF, data);

        BSP::OS::Delay(1);
    }
}
