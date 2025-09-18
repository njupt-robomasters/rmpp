#include "app.hpp"

extern "C" void task_gimbal_entry(const void* argument) {
    gimbal.WaitReady();
    while (true) {
        if (dj6.is_connected) {
            gimbal.SetEnable(true);

            switch (dj6.right_switch) {
            case DJ6::UP:
                gimbal.SetMode(Gimbal::ECD_MODE);
                break;
            case DJ6::MID:
                gimbal.SetMode(Gimbal::IMU_MODE);
                break;
            case DJ6::DOWN:
                gimbal.SetMode(Gimbal::IMU_MODE);
                break;
            }

            gimbal.SetChassisVR(-chassis.vr.measure);

            const Unit<deg_s> yaw_speed = dj6.yaw * app_params.yaw_max_speed;
            const Unit<deg_s> pitch_speed = dj6.pitch * app_params.pitch_max_speed;
            gimbal.SetSpeed(yaw_speed, pitch_speed);
        } else {
            gimbal.SetEnable(false);
        }

        gimbal.Update();
        BSP::OS::Delay(1);
    }
}
