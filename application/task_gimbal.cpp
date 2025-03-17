#include "task_gimbal.h"
#include "cmsis_os.h"
#include "app_variable.hpp"
#include "bsp_can.h"
#include "bsp_dwt.h"

// PITCH最大范围
constexpr float PITCH_MAX_RANGE = 50.0f;

// 满杆量速度
constexpr float pitch_max_speed = 30.0f; // 【单位：度/s】
constexpr float yaw_max_speed = 120.0f; // 【单位：度/s】

float pitch_min_angle;
float pitch_angle, yaw_angle, shoot_freq;
uint32_t dwt_tick;

[[noreturn]] void task_gimbal_entry(void const *argument) {
    gimbal.Init();

    while (gimbal.pitch_angle == 0) {
        // 等待收到电机CAN反馈的位置
        osDelay(1);
    }
    pitch_min_angle = gimbal.pitch_angle;
    pitch_angle = pitch_min_angle;

    while (gimbal.yaw_angle == 0) {
        // 等待收到电机CAN反馈的位置
        osDelay(1);
    }
    yaw_angle = gimbal.yaw_angle;

    shoot_freq = 0;

    while (true) {
        const float dt = BSP_DWT_GetDeltaT(&dwt_tick);

        // 检查遥控器连接
        if (dj6.is_connected == false) {
            pitch_angle = gimbal.pitch_angle;
            yaw_angle = gimbal.yaw_angle;
            shoot_freq = 0;
            gimbal.Release();
            // gimbal.Update(pitch_angle, yaw_angle, shoot_freq);
            osDelay(1);
            continue;
        }

        // pitch电机
        pitch_angle += dj6.pitch * pitch_max_speed * dt;
        if (pitch_angle > pitch_min_angle + PITCH_MAX_RANGE)
            pitch_angle = pitch_min_angle + PITCH_MAX_RANGE;
        if (pitch_angle < pitch_min_angle)
            pitch_angle = pitch_min_angle;

        // yaw电机
        yaw_angle += -dj6.yaw * yaw_max_speed * dt;
        if (yaw_angle > 360)
            yaw_angle -= 360;
        if (yaw_angle < 0)
            yaw_angle += 360;

        gimbal.Update(pitch_angle, yaw_angle, shoot_freq);

        osDelay(1);
    }
}

void task_gimbal_callback(const uint32_t id, uint8_t data[8]) {
    gimbal.ParseCAN(id, data);
}
