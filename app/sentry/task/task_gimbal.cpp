#include "app.hpp"

extern "C" void task_gimbal_entry(void* argument) {
    // 等待IMU上线
    while (!imu.is_ready) {
        BSP::OS::Delay(1);
    }

    // 等待IMU稳定
    BSP::OS::Delay(100);

    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            gimbal.SetEnable(false);
        } else {
            gimbal.SetEnable(true);
        }

        // 设置云台速度
        gimbal.SetSpeed(var.yaw_speed.sum, var.pitch_speed.sum);

        // 设置小陀螺前馈
        gimbal.SetChassisVR(chassis.vr.measure);

        gimbal.OnLoop();

        BSP::OS::Delay(1);
    }
}