#include "app.hpp"

extern "C" [[noreturn]] void task_imu_entry(const void* argument) {
    imu.Init();
    // imu.Calibrate();
    while (true) {
        imu.Update();
        BSP::OS::Delay(1);
    }
}
