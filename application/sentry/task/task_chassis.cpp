#include "task_chassis.h"
#include "../app.hpp"

static uint32_t dwt_cnt;
static float dt;

[[noreturn]] void task_chassis_entry(void const *argument) {
    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接
        if (dj6.is_connected == false) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
            osDelay(1);
            continue;;
        }

        const float vx = dj6.x * settings.chassis_vxy_max;
        const float vy = dj6.y * settings.chassis_vxy_max;
        const float vr_rpm = dj6.yaw * settings.chassis_vr_rpm;

        chassis.SetEnable(true);
        chassis.SetPowerLimit(120);
        // chassis.SetPowerLimit(referee.chassis_power_limit);
        chassis.SetSpeed(vx, vy, vr_rpm);

        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
