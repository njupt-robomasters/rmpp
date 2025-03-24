#include "task_imu.h"
#include "../app.hpp"

[[noreturn]] void task_imu_entry(void const *argument) {
    imu.Init();
    while (true) {
        imu.Update();
        osDelay(1);
    }
}
