#include "task_chassis.h"
#include "app.hpp"

static uint32_t dwt_cnt;
static float dt;

static void handle_rc() {
    // 解析遥控器控制
    float vx = dj6.x * settings.chassis_vxy_max;
    float vy = dj6.y * settings.chassis_vxy_max;
    const float vr_rpm = -dj6.yaw * settings.chassis_vr_rpm;

    // 右边摇杆
    if (dj6.right_switch == DJ6::UP) {
        // 开启自动导航
    } else {
        // 关闭自动导航
    }
}

[[noreturn]] void task_chassis_entry(void const *argument) {
    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接
        if (dj6.is_connected == false || not status.ignore_rc_disconnect) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
            osDelay(1);
            continue;;
        }

        chassis.SetEnable(true);
        chassis.SetPowerLimit(referee.chassis_power_limit); // 底盘功率限制
        chassis.SetSpeed(vx, vy, vr_rpm);

        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
