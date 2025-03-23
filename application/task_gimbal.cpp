#include "task_gimbal.h"
#include "cmsis_os.h"
#include "app_variable.hpp"
#include "bsp_dwt.h"

static uint32_t dwt_cnt;
static float dt;

[[noreturn]] void task_gimbal_entry(void const *argument) {
    imu.WaitReady();
    gimbal.Init();

    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接
        if (dj6.is_connected == false) {
            gimbal.SetEnable(false); // 云台失能，关闭所有电机输出
            osDelay(1);
            continue;
        }

        float pitch_angle_add = 0, yaw_angle_add = 0;

        // 解析遥控器操作
        // 1. 云台模式
        if (dj6.right_switch == DJ6::UP) {
            status.gimbal_mode = Gimbal::ECD_MODE; // UP为CDO模式
        } else {
            status.gimbal_mode = Gimbal::IMU_MODE; // MID、DOWN为IMU模式
        }
        // 2. pitch、yaw电机
        pitch_angle_add += dj6.pitch * settings.pitch_aps_max * dt; // pitch电机
        yaw_angle_add += -dj6.yaw * settings.yaw_aps_max * dt; // yaw电机
        // 3. 发射机构
        // 左摇杆控制
        if (dj6.left_switch == DJ6::UP) {
            // 摩擦轮关，拨弹电机关
            status.is_prepare_shoot = false;
            status.is_shoot = false;
        } else if (dj6.left_switch == DJ6::MID) {
            // 摩擦轮开，拨弹电机关
            status.is_prepare_shoot = true;
            status.is_shoot = false;
        } else if (dj6.left_switch == DJ6::DOWN) {
            // 摩擦轮开，拨弹电机开
            status.is_prepare_shoot = true;
            status.is_shoot = true;
        }

        // 解析鼠标操作
        pitch_angle_add += -static_cast<float>(referee.mouse_y) / settings.mouse_y_max * settings.pitch_aps_max * dt;
        yaw_angle_add += -static_cast<float>(referee.mouse_x) / settings.mouse_x_max * settings.yaw_aps_max * dt;
        status.is_shoot |= referee.mouse_left_button_down; // 这里是”或等于“，遥控器和鼠标左键均可开火
        status.is_rv2_mode = referee.mouse_right_button_down;

        // 控制云台
        gimbal.SetEnable(true); // 使能
        gimbal.SetMode(status.gimbal_mode); // 模式
        gimbal.SetYawSpeedFF(-chassis.measure.chassis.vr); // yaw速度前馈
        gimbal.SetShootFreq(status.shoot_freq); // 射频
        gimbal.SetPrepareShoot(status.is_prepare_shoot); // 摩擦轮

        if (status.is_rv2_mode && status.gimbal_mode == Gimbal::IMU_MODE) {
            // 自瞄模式
            if (rv2.is_locked) {
                gimbal.SetAngle(rv2.pitch, rv2.yaw);
            }
            if (rv2.fire_advise && status.is_shoot) {
                gimbal.SetShoot(true);
            } else {
                gimbal.SetShoot(false);
            }
        } else {
            // 手动模式
            gimbal.AddAngle(pitch_angle_add, yaw_angle_add); // 角度增量
            gimbal.SetShoot(status.is_shoot); // 是否开火
        }

        gimbal.Update();

        osDelay(1);
    }
}

void task_gimbal_callback(const uint32_t id, uint8_t data[8]) {
    gimbal.ParseCAN(id, data);
}
