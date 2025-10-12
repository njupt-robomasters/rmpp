#include "app.hpp"
#include "utils.hpp"

struct {
    UnitFloat<deg_s> rc, referee, sum;
} yaw_speed, pitch_speed;

// 解析遥控器操作
static void handle_rc() {
    yaw_speed.rc = dj6.yaw * app_params.yaw_max_speed;
    pitch_speed.rc = dj6.pitch * app_params.pitch_max_speed;

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
static void handle_referee() {
    // yaw_speed.referee = -referee.mouse_x / app_params.mouse_x_max * app_params.yaw_max_speed;
    pitch_speed.referee = -referee.mouse_y / app_params.mouse_y_max * app_params.pitch_max_speed;
}

extern "C" void task_gimbal_entry(const void* argument) {
    gimbal.WaitReady();
    imu.WaitReady();
    BSP::OS::Delay(100); // 等待IMU稳定
    while (true) {
        handle_rc(); // 解析遥控器
        handle_referee(); // 解析图传链路键盘鼠
        // 合并遥控器和键盘控制
        yaw_speed.sum = clamp(yaw_speed.rc + yaw_speed.referee, app_params.yaw_max_speed);
        pitch_speed.sum = clamp(pitch_speed.rc + pitch_speed.referee, app_params.pitch_max_speed);
        gimbal.SetSpeed(yaw_speed.sum, pitch_speed.sum);

        // 检查遥控器断联
        if (dj6.is_connected == false) {
            gimbal.SetEnable(false); // 云台失能
        } else {
            gimbal.SetEnable(true); // 云台使能
        }

        gimbal.SetChassisVR(-chassis.vr.measure); // 设置小陀螺前馈
        gimbal.Update();
        BSP::OS::Delay(1);
    }
}
