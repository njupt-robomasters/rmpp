#include "task_chassis.h"
#include "../app.hpp"

static uint32_t dwt_cnt;
static float dt;

static float vx_keyboard = 0, vy_keyboard = 0;
static uint8_t q_pressed = 0, e_pressed = 0,f_pressed = 0, r_pressed = 0;
static uint16_t r_count = 0;

// 解析遥控器操作
static void handle_rc() {
    status.chassis.rc.vx = dj6.x * status.chassis.vxy_max;
    status.chassis.rc.vy = dj6.y * status.chassis.vxy_max;

    // 解析遥控器
    if (dj6.right_switch == DJ6::UP or dj6.right_switch == DJ6::MID) {
        // UP，MID：小陀螺关闭
        status.chassis.is_turning = 0;
    } else {
        // DOWN：小陀螺开启
        status.chassis.is_turning = 1;
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
    //按下e增加rpm
    if(referee.e){
        if(e_pressed==0){
            e_pressed = 1;
            if(status.chassis.vr_rpm<100)
                status.chassis.vr_rpm+=20;
        }
    }
    else
        e_pressed = 0;

    //按下q减少rpm
    if(referee.q){
        if(q_pressed==0){
            q_pressed = 1;
            if(status.chassis.vr_rpm>20)
                status.chassis.vr_rpm-=20;
        }
    }
    else
        q_pressed = 0;

    //F开启小陀螺
    if(referee.f){
        if(f_pressed==0){
            f_pressed = 1;
            status.chassis.is_turning = !status.chassis.is_turning;//开启或者关闭小陀螺
        }
    }
    else
        f_pressed = 0;

    //R强制使用键盘，忽略一切遥控器信号
    if(referee.r){
        r_count++;
        if(r_count>3000)//按住三秒
            status.is_force_keyboard = 1;
    }
    else
        r_count = 0;

}

[[noreturn]] void task_chassis_entry(void const *argument) {
    imu.WaitReady();

    while (true) {
        dt = BSP_DWT_GetDeltaT(&dwt_cnt);

        // 检查遥控器连接,以及是否强制启用键盘
        if (dj6.is_connected == false && !status.is_force_keyboard) {
            chassis.SetEnable(false); // 底盘失能，关闭电机输出
            osDelay(1);
            continue;
        }
        // 合并遥控器和键盘控制
        handle_rc();
        handle_video();

        const float vx = clamp(status.chassis.rc.vx + status.chassis.video.vx, status.chassis.vxy_max);
        const float vy = clamp(status.chassis.rc.vy + status.chassis.video.vy, status.chassis.vxy_max);

        // 控制底盘
        chassis.SetEnable(true);
        chassis.SetGimbalAngle_RefByChassis(gimbal.measure.yaw.relative); // 使能
        chassis.SetPowerLimit(referee.chassis_power_limit); // 功率控制
        chassis.SetSpeed(vx, vy, status.chassis.is_turning?status.chassis.vr_rpm:0); // 根据小陀螺状态设置

        chassis.Update();
        osDelay(1);
    }
}

void task_chassis_callback(const uint32_t id, uint8_t data[8]) {
    chassis.ParseCAN(id, data);
}
