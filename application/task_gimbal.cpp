#include "task_gimbal.h"
#include "cmsis_os.h"
#include "app_variable.hpp"

[[noreturn]] void task_gimbal_entry(void const *argument) {
    gimbal.Init();
    while (true) {
        gimbal.Update(0, 0, 7.5f);
        // gimbal.Release();
        osDelay(1);
    }
}

void task_gimbal_callback(const uint32_t id, uint8_t data[8]) {
    gimbal.ParseCAN(id, data);
}
