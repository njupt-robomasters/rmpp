#include "task_chassis.h"
#include "cmsis_os.h"
#include "app_variable.hpp"

[[noreturn]] void task_chassis_entry(void *argument) {
    chassis.Init();
    while (true) {
        // 检查遥控器连接
        if (dj6.is_connected == false) {
            chassis.Release();
            osDelay(1);
            continue;
        }

        // 根据运动模式，计算底盘速度分量
        float vx, vy, vr;
        if (dj6.right_switch == DJ6::UP) {
            // 前后移动、左右旋转
            vx = 0;
            vy = dj6.y * settings.chassis_vy_max;
            vr = -dj6.x * settings.chassis_vr_max;
        } else {
            // 前后左右平移
            vx = dj6.x * settings.chassis_vx_max;
            vy = dj6.y * settings.chassis_vy_max;
            vr = 0;
        }
        chassis.Update(vx, vy, vr);

        osDelay(1);
    }
}

void task_chassis_callback(uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
