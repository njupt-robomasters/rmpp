#include "app.hpp"
#include "utils.hpp"

static UnitFloat<deg_s> yaw_speed_rc, pitch_speed_rc;
static UnitFloat<deg_s> yaw_speed_video, pitch_speed_video;

// 解析遥控器操作
static void handle_rc() {
    yaw_speed_rc = dj6.yaw * app_params.yaw_max_speed;
    pitch_speed_rc = dj6.pitch * app_params.pitch_max_speed;

    switch (dj6.right_switch) {
    case DJ6::UP:
        gimbal.SetMode(Gimbal::ECD_MODE);
        break;
    case DJ6::MID:
        gimbal.SetMode(Gimbal::IMU_MODE);
        break;
    case DJ6::DOWN:
        gimbal.SetMode(Gimbal::IMU_MODE);
        break;
    }
}

// 解析鼠标操作
static void handle_video() {
    yaw_speed_video = -referee.mouse_x / app_params.mouse_x_max * app_params.yaw_max_speed;
    pitch_speed_video = -referee.mouse_y / app_params.mouse_y_max * app_params.pitch_max_speed;
}

extern "C" void task_gimbal_entry(const void* argument) {
    gimbal.WaitReady();
    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            gimbal.SetEnable(false);
            gimbal.Update();
            BSP::OS::Delay(1);
            continue;
        }

        // 使能云台
        gimbal.SetEnable(true);

        // 设置小陀螺前馈
        gimbal.SetChassisVR(-chassis.vr.measure);

        handle_rc(); // 解析遥控器
        handle_video(); // 解析图传链路键盘鼠

        // 合并遥控器和键盘控制
        const UnitFloat<deg_s> yaw_speed = clamp(yaw_speed_rc + yaw_speed_video, -app_params.yaw_max_speed, app_params.yaw_max_speed);
        const UnitFloat<deg_s> pitch_speed = clamp(pitch_speed_rc + pitch_speed_video, -app_params.pitch_max_speed, app_params.pitch_max_speed);
        gimbal.SetSpeed(yaw_speed, pitch_speed);

        gimbal.Update();
        BSP::OS::Delay(1);
    }
}
