#include "app.hpp"
#include "utils.hpp"

static UnitFloat<m_s> vx_rc, vy_rc, vx_video, vy_video;
static UnitFloat<rpm> vr;

// 解析遥控器操作
static void handle_rc() {
    vx_rc = dj6.x * app_params.vxy_max;
    vy_rc = dj6.y * app_params.vxy_max;

    if (dj6.right_switch == DJ6::DOWN) { // DOWN：测试小陀螺
        vr = app_params.vr_max;
    } else {
        vr = 0;
    }
}

// 解析键盘操作
static void handle_video() {
    static BSP::Dwt dwt;
    const float dt = dwt.GetDT();

    // 前进后退
    if (referee.w) {
        vx_video = vx_video + app_params.axy * dt;
    } else if (referee.s) {
        vx_video = vx_video - app_params.axy * dt;
    } else {
        vx_video = 0;
    }
    vx_video = clamp(vx_video, -app_params.vxy_max, app_params.vxy_max);
    // 左右平移
    if (referee.a) {
        vy_video = vy_video + app_params.axy * dt;
    } else if (referee.d) {
        vy_video = vy_video - app_params.axy * dt;
    } else {
        vy_video = 0;
    }
    vy_video = clamp(vy_video, -app_params.vxy_max, app_params.vxy_max);
}

extern "C" void task_chassis_entry(const void* argument) {
    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            chassis.SetEnable(false);
            chassis.Update();
            BSP::OS::Delay(1);
            continue;
        }

        // 使能底盘
        chassis.SetEnable(true);

        // 设置云台方向
        chassis.SetGimbalYaw(gimbal.yaw.relative.measure);

        handle_rc(); // 解析遥控器
        handle_video(); // 解析图传链路键盘鼠

        // 合并遥控器和键盘控制
        const UnitFloat<m_s> vx = clamp(vx_rc + vx_video, -app_params.vxy_max, app_params.vxy_max);
        const UnitFloat<m_s> vy = clamp(vy_rc + vy_video, -app_params.vxy_max, app_params.vxy_max);
        chassis.SetSpeed(vx, vy, vr);

        chassis.Update();
        BSP::OS::Delay(1);
    }
}
