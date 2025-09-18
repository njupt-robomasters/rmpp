#include "app.hpp"

extern "C" void task_chassis_entry(const void* argument) {
    while (true) {
        if (dj6.is_connected) {
            chassis.SetEnable(true);

            chassis.SetGimbalYaw(gimbal.yaw.relative.measure);

            Unit<m_s> vx = dj6.x * app_params.vxy_max;
            Unit<m_s> vy = dj6.y * app_params.vxy_max;
            Unit<rpm> vr;
            if (dj6.right_switch == DJ6::DOWN) {
                vr = app_params.vr_max;
            } else {
                vr = 0;
            }
            chassis.SetSpeed(vx, vy, vr);
        } else {
            chassis.SetEnable(false);
        }

        chassis.Update();
        BSP::OS::Delay(1);
    }
}
