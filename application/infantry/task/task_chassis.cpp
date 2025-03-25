#include "task_chassis.h"
#include "../app.hpp"
#include "main.h"

static uint32_t dwt_cnt;
static float dt;

static DJ6::switch_t last_right_switch = DJ6::ERR;
static bool last_q = false, last_e = false, last_z = false, last_c = false;
static int r_count = 0, vb_count = 0;

// 解析遥控器操作
static void handle_rc() {
    status.chassis.rc.vx = dj6.x * status.chassis.vxy_limit;
    status.chassis.rc.vy = dj6.y * status.chassis.vxy_limit;

    // 解析遥控器
    if (dj6.right_switch != last_right_switch) {
        last_right_switch = dj6.right_switch;
        if (dj6.right_switch == DJ6::UP or dj6.right_switch == DJ6::MID) {
            // UP，MID：小陀螺关闭
            status.chassis.rpm = 0;
        } else {
            // DOWN：小陀螺开启
            status.chassis.rpm = 60; // 60rpm测试小陀螺
        }
    }
}

// 解析键盘操作
static void handle_video() {
    // shfit+aswd
    if (referee.shift) {
        // 最大加速度模式
        // 前进后退
        if (referee.w) {
            status.chassis.video.vy = status.chassis.vxy_limit;
        } else if (referee.s) {
            status.chassis.video.vy = -status.chassis.vxy_limit;
        } else {
            status.chassis.video.vy = 0;
        }
        // 左右平移
        if (referee.a) {
            status.chassis.video.vx = -status.chassis.vxy_limit;
        } else if (referee.d) {
            status.chassis.video.vx = status.chassis.vxy_limit;
        } else {
            status.chassis.video.vx = 0;
        }
    } else {
        // 前进后退
        if (referee.w) {
            status.chassis.video.vy += settings.axy * dt;
        } else if (referee.s) {
            status.chassis.video.vy -= settings.axy * dt;
        } else {
            status.chassis.video.vy = 0;
        }
        // 左右平移
        if (referee.a) {
            status.chassis.video.vx -= settings.axy * dt;
        } else if (referee.d) {
            status.chassis.video.vx += settings.axy * dt;
        } else {
            status.chassis.video.vx = 0;
        }
    }

    // E增加小陀螺转速
    if (referee.e != last_e) {
        last_e = referee.e;
        if (referee.e) {
            status.chassis.rpm += settings.rpm_per_press;
            status.chassis.rpm = clamp(status.chassis.rpm, 0, settings.rpm_max);
        }
    }

    // Q减少小陀螺转速
    if (referee.q != last_q) {
        last_q = referee.q;
        if (referee.q) {
            status.chassis.rpm -= settings.rpm_per_press;
            status.chassis.rpm = clamp(status.chassis.rpm, 0, settings.rpm_max);
        }
    }

    // C增大移动速度
    if (referee.c != last_c) {
        last_c = referee.c;
        if (referee.c) {
            status.chassis.vxy_limit += settings.vxy_per_press;
            status.chassis.vxy_limit = clamp(status.chassis.vxy_limit, 0, settings.vxy_max);
        }
    }

    // Z减少移动速度
    if (referee.z != last_z) {
        last_z = referee.z;
        if (referee.z) {
            status.chassis.vxy_limit -= settings.vxy_per_press;
            status.chassis.vxy_limit = clamp(status.chassis.vxy_limit, 0, settings.vxy_max);
        }
    }

    // R强制使用键盘，忽略遥控器断连
    if (referee.r) {
        r_count++;
        if (r_count > 3000) // 按住三秒
            status.ignore_rc_disconnect = true;
    } else {
        r_count = 0;
    }

    // // 同时按VB三秒重启C板
    // if (referee.v && referee.b) {
    //     vb_count++;
    //     if (vb_count > 3000) // 按住三秒
    //         NVIC_SystemReset();
    // } else {
    //     vb_count = 0;
    // }
}

[[noreturn]] void task_chassis_entry(void const *argument) {
    imu.WaitReady();

    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        handle_rc();
        handle_video();

        // 检查遥控器连接,以及是否强制启用键盘
        if (dj6.is_connected == false && !status.ignore_rc_disconnect) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
            osDelay(1);
            continue;
        }

        // 合并遥控器和键盘控制
        const float vx = clamp(status.chassis.rc.vx + status.chassis.video.vx, status.chassis.vxy_limit);
        const float vy = clamp(status.chassis.rc.vy + status.chassis.video.vy, status.chassis.vxy_limit);

        // 控制底盘
        chassis.SetEnable(true); // 使能
        chassis.SetGimbalAngle_RefByChassis(gimbal.measure.yaw.relative);
        if (superCapacity.isLowEnergy()) {
            chassis.SetPowerLimit(referee.chassis_power_limit); // 主动功率控制
        } else {
            chassis.SetPowerLimit(200);
        }
        chassis.SetSpeed(vx, vy, status.chassis.rpm);
        superCapacity.SendPowerLimit(referee.chassis_power_limit - 5);

        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
    superCapacity.ParseCAN(id, data);
}
