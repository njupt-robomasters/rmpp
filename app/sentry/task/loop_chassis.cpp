#include "app.hpp"

void loop_chassis() {
    // 检查遥控器断联
    if (dj6.is_connected == false) {
        chassis.SetEnable(false);
    } else {
        chassis.SetEnable(true);
    }

    // 设置底盘速度
    chassis.SetSpeed(var.vx.sum, var.vy.sum, var.vr.sum);

    // 设置云台方向
    chassis.SetGimbalYaw(gimbal.yaw.ecd.measure);

    chassis.OnLoop();
}