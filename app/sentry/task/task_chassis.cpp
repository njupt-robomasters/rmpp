#include "app.hpp"

// 解析遥控器操作
static void handle_rc() {
    var.vx.rc = dj6.x * cfg.vxy_max;
    var.vy.rc = dj6.y * cfg.vxy_max;

    // 测试小陀螺
    if (dj6.right_switch == DJ6::DOWN) {
        var.vr.rc = cfg.vr_max;
    } else {
        var.vr.rc = 0 * default_unit;
    }
}

extern "C" void task_chassis_entry(void* argument) {
    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            chassis.SetEnable(false); // 底盘失能
        } else {
            chassis.SetEnable(true); // 底盘使能
        }

        handle_rc();
        var.vx.sum = unit::clamp(var.vx.rc + var.vx.referee, cfg.vxy_max);
        var.vy.sum = unit::clamp(var.vy.rc + var.vy.referee, cfg.vxy_max);
        var.vr.sum = unit::clamp(var.vr.rc + var.vr.referee, cfg.vr_max);
        chassis.SetSpeed(var.vx.sum, var.vy.sum, var.vr.sum);

        chassis.SetGimbalYaw(gimbal.yaw.ecd.measure); // 设置云台方向
        chassis.OnLoop();
        BSP::OS::Delay(1);
    }
}