#include "gimbal.hpp"
#include "app.hpp"

static constexpr float SPEED_MAX = 60 * rpm; // 摇杆满量程对应的云台转速

static Pid yaw_pid(&yaw_pid_config);

// 观察量：云台世界系朝向目标与测量
static Angle yaw_ref;
static Angle yaw_measure;

void gimbal_loop() {
    attitude.update();

    // IMU 在底盘上：云台朝向 = 底盘航向 + 电机相对底盘角度
    yaw_measure = attitude.yaw + yaw.position;

    // SWA 下挡且电机在线时使能；未使能时目标跟随测量并清空 PID
    static Timer yaw_speed_timer;
    const float dt = yaw_speed_timer.restart();
    const bool is_enable = fsi6x.is_connect && fsi6x.swa == FSi6X::DOWN && yaw.is_connect;
    yaw.set_enable(is_enable);
    if (!is_enable) {
        yaw_ref = yaw_measure;
        yaw_pid.clear();
        yaw.set_current(0);
    } else {
        // 跟随（SWB 下）或小陀螺（SWD 下）时左摇杆水平控制云台转速，否则保持世界系朝向
        if (fsi6x.swb == FSi6X::DOWN || fsi6x.swd == FSi6X::DOWN)
            yaw_ref += fsi6x.yaw * SPEED_MAX * dt;
        yaw.set_current(yaw_pid.calculate(Angle(yaw_ref - yaw_measure)));
    }
    yaw.update();

    // RMDL7025 两种报文合计应答上限 500 Hz
    static Timer yaw_send_timer;
    if (yaw_send_timer.elapsed() >= 0.002f) {
        yaw_send_timer.restart();
        yaw.send();
    }
}
