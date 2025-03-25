#include "task_chassis.h"
#include "../app.hpp"

static uint32_t dwt_cnt;
static float dt;

static float vx_keyboard = 0, vy_keyboard = 0;

[[noreturn]] void task_chassis_entry(void const *argument) {
    imu.WaitReady();

    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接
        if (dj6.is_connected == false) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
            osDelay(1);
            continue;;
        }

        float vx = 0, vy = 0, vr_rpm = 0;

        vx = dj6.x * status.chassis_vxy_max;
        vy = dj6.y * status.chassis_vxy_max;

        // 解析遥控器
        if (dj6.right_switch == DJ6::UP or dj6.right_switch == DJ6::MID) {
            // UP，MID：小陀螺关闭
            vr_rpm = 0;
        } else {
            // DOWM：小陀螺开启
            vr_rpm = status.chassis_vr_rpm;
        }

        // 解析键盘操作
        if (referee.keyboard_value & (1 << 0)) {
            // W
            vy_keyboard += status.chassis_avy * dt;
        } else if (referee.keyboard_value & (1 << 1)) {
            // S
            vy_keyboard -= status.chassis_avy * dt;
        } else {
            vy_keyboard = 0;
        }
        if (referee.keyboard_value & (1 << 2)) {
            // A
            vx_keyboard -= status.chassis_avy * dt;
        } else if (referee.keyboard_value & (1 << 3)) {
            // D
            vx_keyboard += status.chassis_avy * dt;
        } else {
            vx_keyboard = 0;
        }

        // 合并遥控器和键盘控制
        vx += vx_keyboard;
        vx = clamp(vx, status.chassis_vxy_max);
        vy += vy_keyboard;
        vy = clamp(vy, status.chassis_vxy_max);

        chassis.SetEnable(true);
        chassis.SetPowerLimit(60);
        // chassis.SetPowerLimit(referee.chassis_power_limit);
        chassis.SetGimbalAngle_RefByChassis(gimbal.measure.yaw.relative);
        chassis.SetSpeed(vx, vy, vr_rpm);


        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
