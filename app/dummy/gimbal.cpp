#include "gimbal.hpp"
#include "app.hpp"
#include <algorithm>

static constexpr float SPEED_MAX = 60 * rpm;             // 摇杆满量程对应的云台转速
static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s

static Pid yaw_pid(&yaw_pid_config);
static Timer update_timer;
static uint32_t yaw_send_count = 0; // yaw 发送分频计数

// 观察量：云台世界系朝向目标与测量
static Angle yaw_ref;
static Angle yaw_measure;

void gimbal_loop() {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);
    attitude.update();

    // IMU 在底盘上：云台朝向 = 底盘航向 + 电机相对底盘角度
    yaw_measure = attitude.yaw + yaw.position;

    // SWA 下挡且电机在线时使能；未使能时目标跟随测量并清空 PID
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
        yaw.set_current(yaw_pid.calculate(dt, Angle(yaw_ref - yaw_measure), -yaw.speed));
    }
    yaw.update();

    // RMDL7025 两种报文合计应答上限 500 Hz
    if (yaw_send_count++ % 2 == 0) {
        yaw.send();
    }
}
