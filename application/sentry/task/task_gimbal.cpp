#include "app.hpp"

extern "C" void task_gimbal_entry(const void* argument) {
    gimbal.WaitReady();
    while (true) {
        if (dj6.is_connected) {
            gimbal.SetEnable(true);
        } else {
            gimbal.SetEnable(false);
        }

        gimbal.Update();
        BSP::OS::Delay(1);
    }
}
