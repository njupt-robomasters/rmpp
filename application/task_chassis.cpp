#include "task_chassis.h"
#include "cmsis_os.h"
#include "app_variable.hpp"

[[noreturn]] void task_chassis_entry(void *argument) {
    imu.WaitReady();

    while (true) {
        // 检查遥控器连接
        if (dj6.is_connected == false) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
        } else {
            chassis.SetEnable(true);

            // 根据运动模式，计算底盘速度分量
            float vx_mps, vy_mps, vr_tps;
            if (dj6.right_switch == DJ6::UP or dj6.right_switch == DJ6::MID) {
                // 前后左右平移
                vx_mps = dj6.x * settings.chassis_vx_mps_max;
                vy_mps = dj6.y * settings.chassis_vy_mps_max;
                vr_tps = 0;
            } else { // DOWN
                // 前后移动、左右旋转
                vx_mps = 0;
                vy_mps = dj6.y * settings.chassis_vy_mps_max;
                vr_tps = -dj6.x * settings.chassis_vr_tps_max;
            }

            chassis.SetSpeed(vx_mps, vy_mps, vr_tps);

            chassis.SetGimbalRefByChassisAngle(gimbal.yaw_angle_measure_absolute);
        }

        chassis.Update();

        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
