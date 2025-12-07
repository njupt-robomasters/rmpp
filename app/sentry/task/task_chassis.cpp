#include "app.hpp"


extern "C" void task_chassis_entry(void* argument) {
    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            chassis.SetEnable(false);
        } else {
            chassis.SetEnable(true);
        }

        chassis.SetSpeed(var.vx.sum, var.vy.sum, var.vr.sum);

        // 设置云台方向
        chassis.SetGimbalYaw(gimbal.yaw.ecd.measure);

        chassis.OnLoop();

        BSP::OS::Delay(1);
    }
}