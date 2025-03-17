#include "task_chassis.h"
#include "cmsis_os.h"
#include "app_variable.hpp"

// 满杆量速度
constexpr float vx_max = 1.0f; // 左右平移速度【单位：m/s】
constexpr float vy_max = 1.0f; // 前进后退速度【单位：m/s】
constexpr float vr_max = 10.0f; // 旋转角速度【单位：rpm】

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
            vy = dj6.y * vy_max;
            vr = -dj6.x * vr_max;
        } else {
            // 前后左右平移
            vx = dj6.x * vx_max;
            vy = dj6.y * vy_max;
            vr = 0;
        }
        chassis.Update(vx, vy, vr);

        osDelay(1);
    }
}

void task_chassis_callback(uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
