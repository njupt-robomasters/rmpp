#include "task_gimbal.h"
#include "cmsis_os.h"
#include "app_variable.hpp"
#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "utils.hpp"

// PITCH最大角度
constexpr float PITCH_MAX_ANGLE = 50.0f;

float pitch_angle, yaw_angle, shoot_freq;
uint32_t dwt_cnt;

[[noreturn]] void task_gimbal_entry(void const *argument) {
    gimbal.WaitPitchYawStartup(); // 等待pitch yaw电机启动
    gimbal.PitchZero(); // pitch回零

    // 初始值
    pitch_angle = 0;
    yaw_angle = gimbal.yaw_angle;
    shoot_freq = 0;

    while (true) {
        const float dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接
        if (dj6.is_connected == false) {
            // 不断更新当前位置到设定位置，防止遥控器恢复后剧烈运动
            pitch_angle = gimbal.pitch_angle;
            yaw_angle = gimbal.yaw_angle;
            shoot_freq = 0;

            // 释放云台
            gimbal.Release();

            // 关闭摩擦轮电机
            BSP_PWM_SetDuty(0);

            osDelay(1);
            continue;
        }

        // pitch电机
        pitch_angle += dj6.pitch * settings.pitch_max_speed * dt;
        clamp(pitch_angle, 0, PITCH_MAX_ANGLE);

        // yaw电机
        yaw_angle += -dj6.yaw * settings.yaw_max_speed * dt;
        yaw_angle = norm_angle(yaw_angle);

        // shoot拨弹电机
        if (dj6.left_switch == DJ6::UP) {
            shoot_freq = 0;
            BSP_PWM_SetDuty(0);
        } else if (dj6.left_switch == DJ6::MID) {
            shoot_freq = 0;
            BSP_PWM_SetDuty(1);
        } else if (dj6.left_switch == DJ6::DOWN) {
            shoot_freq = settings.shoot_freq;
            BSP_PWM_SetDuty(1);
        }

        gimbal.Update(pitch_angle, yaw_angle, shoot_freq);

        osDelay(1);
    }
}

void task_gimbal_callback(const uint32_t id, uint8_t data[8]) {
    gimbal.ParseCAN(id, data);
}
