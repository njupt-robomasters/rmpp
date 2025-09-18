#include "app.hpp"

extern "C" void task_chassis_entry(const void* argument) {
    while (true) {
        if (dj6.is_connected) {
            chassis.SetEnable(true);
            Unit<m_s> vx = dj6.x * settings.vxy_max;
            Unit<m_s> vy = dj6.y * settings.vxy_max;
            Unit<rpm> vr = dj6.yaw * settings.vr_max;
            chassis.SetSpeed(vx, vy, vr);
        } else {
            chassis.SetEnable(false);
        }

        chassis.Update();
        BSP::OS::Delay(1);
    }
}
