#include "task_gimbal.h"
#include "cmsis_os.h"
#include "app_variable.hpp"
#include "bsp_dwt.h"

uint32_t dwt_cnt;
float dt;

[[noreturn]] void task_gimbal_entry(void const *argument) {
    imu.WaitReady();
    gimbal.Init();

    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接
        if (dj6.is_connected == false) {
            gimbal.SetEnable(false); // 云台失能，关闭所有电机输出
        } else {
            gimbal.SetEnable(true);

            if (dj6.right_switch == DJ6::UP) {
                gimbal.SetYawMode(Gimbal::ECD_MODE);
            } else {
                // MID、DOWN
                gimbal.SetYawMode(Gimbal::IMU_MODE);
                gimbal.SetYawSpeedFF(-chassis.measure.chassis.vr);
            }

            // pitch、yaw电机
            const float pitch_angle_add = dj6.pitch * settings.pitch_aps_max * dt; // pitch电机
            const float yaw_angle_add = -dj6.yaw * settings.yaw_aps_max * dt; // yaw电机
            gimbal.AddAngle(pitch_angle_add, yaw_angle_add);

            // shoot拨弹电机
            // 左摇杆控制
            if (dj6.left_switch == DJ6::UP) {
                gimbal.SetPrepareShoot(false);
                gimbal.SetShootFreq(0);
            } else if (dj6.left_switch == DJ6::MID) {
                gimbal.SetPrepareShoot(true);
                gimbal.SetShootFreq(0);
            } else if (dj6.left_switch == DJ6::DOWN) {
                gimbal.SetPrepareShoot(true);
                gimbal.SetShootFreq(settings.shoot_freq);
            }

            // gimbal.SetEnable(true);
            // gimbal.SetYawMode(Gimbal::IMU_MODE);
            // gimbal.SetAngle(rv2.pitch, rv2.yaw);

            // gimbal.SetEnable(false);
        }

        gimbal.Update();

        osDelay(1);
    }
}

void task_gimbal_callback(const uint32_t id, uint8_t data[8]) {
    gimbal.ParseCAN(id, data);
}
