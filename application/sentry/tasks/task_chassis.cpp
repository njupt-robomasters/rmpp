#include "app.hpp"
#include "utils.hpp"

struct {
    UnitFloat<m_s> rc, referee, sum;
} vx, vy;

struct {
    UnitFloat<rpm> rc, referee, sum;
} vr;

// 解析遥控器操作
static void handle_rc() {
    vx.rc = dj6.x * app_params.vxy_max;
    vy.rc = dj6.y * app_params.vxy_max;
    // vr.rc = dj6.yaw * app_params.vr_max;

    // if (dj6.right_switch == DJ6::DOWN) { // DOWN：测试小陀螺
    //     vr.rc = app_params.vr_max;
    // } else {
    //     vr.rc = 0;
    // }
}

// 解析键盘操作
static void handle_referee() {
    static BSP::Dwt dwt;
    const float dt = dwt.GetDT();

    // 前进后退
    if (referee.w) {
        vx.referee += app_params.axy * dt;
    } else if (referee.s) {
        vx.referee -= app_params.axy * dt;
    } else {
        vx.referee = 0;
    }
    vx.referee = clamp(vx.referee, app_params.vxy_max);

    // 左右平移
    if (referee.a) {
        vy.referee += app_params.axy * dt;
    } else if (referee.d) {
        vy.referee -= app_params.axy * dt;
    } else {
        vy.referee = 0;
    }
    vy.referee = clamp(vy.referee, app_params.vxy_max);

    // 底盘旋转
    vr.referee = -referee.mouse_x / app_params.mouse_x_max * app_params.vr_max;
}

extern "C" void task_chassis_entry(const void* argument) {
    while (true) {
        handle_rc(); // 解析遥控器
        handle_referee(); // 解析图传链路键鼠
        // 合并遥控器和键盘控制
        vx.sum = clamp(vx.rc + vx.referee, app_params.vxy_max);
        vy.sum = clamp(vy.rc + vy.referee, app_params.vxy_max);
        vr.sum = clamp(vr.rc + vr.referee, app_params.vr_max);
        chassis.SetSpeed(vx.sum, vy.sum, vr.sum);

        // 检查遥控器断联
        if (dj6.is_connected == false) {
            chassis.SetEnable(false); // 底盘失能
        } else {
            chassis.SetEnable(true); // 底盘使能
        }

        chassis.SetGimbalYaw(gimbal.yaw.relative.measure); // 设置云台方向
        chassis.Update();
        BSP::OS::Delay(1);
    }
}
