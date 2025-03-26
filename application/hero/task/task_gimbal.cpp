#include "task_gimbal.h"
#include "../app.hpp"

static uint32_t dwt_cnt;
static float dt;

static DJ6::switch_t last_right_switch = DJ6::ERR;
static DJ6::switch_t last_left_switch = DJ6::ERR;
static bool last_mouse_left = false;

// 解析遥控器操作
static void handle_rc() {
    // 1. 云台pitch、yaw电机
    // 遥控和键盘叠加控制
    status.gimbal.rc.pitch_speed = dj6.pitch * settings.pitch_aps_max;
    status.gimbal.rc.yaw_speed = -dj6.yaw * settings.yaw_aps_max;

    // 2. 右拨杆控制云台模式
    // 只响应变化量
    if (dj6.right_switch != last_right_switch) {
        last_right_switch = dj6.right_switch;
        if (dj6.right_switch == DJ6::UP) {
            status.gimbal.mode = Gimbal::ECD_MODE; // UP为CDO模式
        } else {
            status.gimbal.mode = Gimbal::IMU_MODE; // MID、DOWN为IMU模式
        }
    }

    // 3. 左摇杆控制发射机构
    // 只响应变化量
    if (dj6.left_switch != last_left_switch) {
        last_left_switch = dj6.left_switch;
        if (dj6.left_switch == DJ6::UP) {
            // 摩擦轮关，拨弹电机关
            status.gimbal.is_prepare_shoot = false;
            status.gimbal.is_shoot = false;
        } else if (dj6.left_switch == DJ6::MID) {
            // 摩擦轮开，拨弹电机关
            status.gimbal.is_prepare_shoot = true;
            status.gimbal.is_shoot = false;
        } else if (dj6.left_switch == DJ6::DOWN) {
            // 摩擦轮开，拨弹电机开
            status.gimbal.is_prepare_shoot = true;
            status.gimbal.is_shoot = true;
        }
    }
}

// 解析鼠标操作
static void handle_video() {
    // 1. 云台pitch、yaw电机
    // 遥控和键盘叠加控制
    status.gimbal.video.pitch_speed = -static_cast<float>(referee.mouse_y) / settings.mouse_y_max * settings.
                                      pitch_aps_max;
    status.gimbal.video.yaw_speed = -static_cast<float>(referee.mouse_x) / settings.mouse_x_max * settings.yaw_aps_max;

    // 2. 左键开火
    // 只响应变化量
    if (referee.mouse_left_button_down != last_mouse_left) {
        last_mouse_left = referee.mouse_left_button_down;
        status.gimbal.is_shoot = referee.mouse_left_button_down;
    }

    // 3. 右键自瞄
    // 仅在云台IMU模式下允许自瞄
    if (referee.mouse_right_button_down && status.gimbal.mode == Gimbal::IMU_MODE) {
        status.gimbal.is_rv2_mode = true;
    } else {
        status.gimbal.is_rv2_mode = false;
    }
}

[[noreturn]] void task_gimbal_entry(void const *argument) {
    imu.WaitReady();
    gimbal.Init();

    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        handle_rc();
        handle_video();

        // 检查遥控器连接 是否为强制键盘模式
        if (dj6.is_connected == false && !status.ignore_rc_disconnect) {
            gimbal.SetEnable(false); // 云台失能，关闭所有电机输出
            gimbal.Update();
            osDelay(1);
            continue;
        }

        // 控制云台
        gimbal.SetEnable(true); // 使能
        gimbal.SetMode(status.gimbal.mode); // 模式
        gimbal.SetYawSpeedFF(-chassis.measure.chassis.vr); // yaw速度前馈
        gimbal.SetPrepareShoot(status.gimbal.is_prepare_shoot); // 摩擦轮

        // 手动模式
        // 合并遥控器和键鼠控制
        const float pitch_angle_add = (status.gimbal.rc.pitch_speed + status.gimbal.video.pitch_speed) * dt;
        const float yaw_angle_add = (status.gimbal.rc.yaw_speed + status.gimbal.video.yaw_speed) * dt;
        gimbal.AddAngle(pitch_angle_add, yaw_angle_add); // 角度增量
        // gimbal.SetShoot(status.gimbal.is_shoot, status.gimbal.shoot_freq); // 是否开火

        gimbal.Update();
        osDelay(1);
    }
}

void task_gimbal_callback(const uint32_t id, uint8_t data[8]) {
    gimbal.ParseCAN(id, data);
}
