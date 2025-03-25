#include "task_chassis.h"
#include "../app.hpp"

static uint32_t dwt_cnt;
static float dt;

static float vx_keyboard = 0, vy_keyboard = 0;


// 解析遥控器操作
static void handle_rc() {
    status.chassis.rc.vx = dj6.x * status.chassis.vxy_max;
    status.chassis.rc.vy = dj6.y * status.chassis.vxy_max;

    // 解析遥控器
    if (dj6.right_switch == DJ6::UP or dj6.right_switch == DJ6::MID) {
        // UP，MID：小陀螺关闭
        status.chassis.vr_rpm = 0;
    } else {
        // DOWM：小陀螺开启
        status.chassis.vr_rpm = 30;
    }
}

// 解析键盘操作
static void handle_video() {
    // 解析键盘操作
    if (referee.shift) {
        // 最大加速度模式
        // 前进后退
        if (referee.w) {
            status.chassis.video.vy = status.chassis.vxy_max;
        } else if (referee.s) {
            status.chassis.video.vy = -status.chassis.vxy_max;
        } else {
            status.chassis.video.vy = 0;
        }
        // 左右平移
        if (referee.a) {
            status.chassis.video.vx = -status.chassis.vxy_max;
        } else if (referee.d) {
            status.chassis.video.vx = status.chassis.vxy_max;
        } else {
            status.chassis.video.vx = 0;
        }
    } else {
        // 前进后退
        if (referee.w) {
            status.chassis.video.vy += status.chassis.avy * dt;
        } else if (referee.s) {
            status.chassis.video.vy -= status.chassis.avy * dt;
        } else {
            status.chassis.video.vy = 0;
        }
        // 左右平移
        if (referee.a) {
            status.chassis.video.vx -= status.chassis.avy * dt;
        } else if (referee.d) {
            status.chassis.video.vx += status.chassis.avy * dt;
        } else {
            status.chassis.video.vx = 0;
        }
    }
}

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

        // 合并遥控器和键盘控制
        const float vx = clamp(status.chassis.rc.vx + status.chassis.video.vx, status.chassis.vxy_max);
        const float vy = clamp(status.chassis.rc.vy + status.chassis.video.vy, status.chassis.vxy_max);

        // 控制底盘
        chassis.SetEnable(true);
        chassis.SetGimbalAngle_RefByChassis(gimbal.measure.yaw.relative); // 使能
        chassis.SetPowerLimit(referee.chassis_power_limit); // 功率控制
        chassis.SetSpeed(vx, vy, status.chassis.vr_rpm); // 设置速度

        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
