#include "app.hpp"

void loop_gimbal() {
    // 检查遥控器断联
    if (dj6.is_connected == false) {
        gimbal.SetEnable(false);
    } else {
        gimbal.SetEnable(true);
    }

    // 设置云台速度
    gimbal.SetSpeed(var.yaw_speed.sum, var.pitch_speed.sum);

    // 设置小陀螺前馈
    gimbal.SetChassisVR(chassis.vr.measure);

    gimbal.OnLoop();
}