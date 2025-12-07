#include "app.hpp"

extern "C" void task_imu_entry(void* argument) {
    imu.Init();
    // imu.Calibrate();
    while (true) {
        imu.OnLoop();
        BSP::OS::Delay(1);
    }
}