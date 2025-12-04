#include "app.hpp"

// 解析遥控器操作
static void handle_rc() {
    var.yaw_speed.rc = dj6.yaw * cfg.yaw_max_speed;
    var.pitch_speed.rc = dj6.pitch * cfg.pitch_max_speed;

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

extern "C" void task_gimbal_entry(void* argument) {
    // 等待云台电机上线
    while (!gimbal.IsReady()) {
        BSP::OS::Delay(1);
    }

    // 等待IMU上线
    while (!imu.is_ready) {
        BSP::OS::Delay(1);
    }

    // 等待IMU稳定
    BSP::OS::Delay(100);

    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            gimbal.SetEnable(false); // 云台失能
        } else {
            gimbal.SetEnable(true); // 云台使能
        }

        handle_rc();
        var.yaw_speed.sum = unit::clamp(var.yaw_speed.rc + var.yaw_speed.referee, cfg.yaw_max_speed);
        var.pitch_speed.sum = unit::clamp(var.pitch_speed.rc + var.pitch_speed.referee, cfg.pitch_max_speed);
        gimbal.SetSpeed(var.yaw_speed.sum, var.pitch_speed.sum);

        gimbal.SetChassisVR(chassis.vr.measure); // 设置小陀螺前馈
        gimbal.OnLoop();
        BSP::OS::Delay(1);
    }
}
