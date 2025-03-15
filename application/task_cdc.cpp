#include "task_cdc.h"

#include "cmsis_os.h"
#include "app_variable.hpp"

[[noreturn]] void task_cdc_entry(void const * argument) {
    while (true) {
        rv2.SendIMUData(imu.roll, imu.pitch, imu.yaw);
        osDelay(10);
    }
}

void task_cdc_callback(const uint8_t *data, const int size) {
    rv2.ParseStreamingData(data, size);
}
