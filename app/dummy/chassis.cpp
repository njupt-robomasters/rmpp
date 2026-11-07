#include "chassis.hpp"
#include "app.hpp"
#include "gimbal.hpp"

// 操控参数
static constexpr float VXY_MAX = 1.0f;     // 平移最大速度，m/s
static constexpr float WR_MAX = 60 * rpm;  // 手动旋转最大角速度
static constexpr float SPIN_WR = 60 * rpm; // 小陀螺角速度

// N630 HFI 静止附近有高频噪音：目标电流与实际轮速均低于阈值时发零电流
static constexpr float LOW_CURRENT = 0.3f; // A
static constexpr float LOW_SPEED = 5 * rpm;

void chassis_loop() {
    // SWA 下挡使能，断联时拨杆为 ERR；SWD 下挡小陀螺；SWB 中、下挡跟随云台
    const bool is_enable = fsi6x.is_connect && fsi6x.swa == FSi6X::DOWN;
    const bool is_spin = fsi6x.swd == FSi6X::DOWN;
    const bool is_follow = fsi6x.swb == FSi6X::MID || fsi6x.swb == FSi6X::DOWN;

    // 遥控：右摇杆平移；小陀螺固定角速度，否则不跟随时左摇杆水平控制底盘旋转
    float vx = 0, vy = 0, wr = 0;
    if (is_enable) {
        vx = fsi6x.x * VXY_MAX;
        vy = fsi6x.y * VXY_MAX;
        if (is_spin)
            wr = SPIN_WR;
        else if (!is_follow)
            wr = -fsi6x.yaw * WR_MAX;
    }

    // 底盘控制：小陀螺或跟随时以云台为前方，小陀螺角速度优先于跟随；云台离线时不跟随，以底盘原始前方
    const bool is_gimbal_front = (is_spin || is_follow) && yaw.is_connect;
    chassis.set_enable(is_enable);
    chassis.set_mode(is_follow && yaw.is_connect ? ChassisMatrix::FOLLOW_MODE : ChassisMatrix::DETACH_MODE);
    chassis.set_gimbal_yaw(is_gimbal_front ? yaw.position : 0.0f);
    chassis.set_speed(vx, vy, wr);

    // 超电模式：SWC 上 MONITOR、中 CHARGE、下 BOOST，遥控断联时 OFF；底盘失能时仍可充电
    if (fsi6x.swc == FSi6X::UP)
        super_cap.set_mode(SuperCap::MONITOR);
    else if (fsi6x.swc == FSi6X::MID)
        super_cap.set_mode(SuperCap::CHARGE);
    else if (fsi6x.swc == FSi6X::DOWN)
        super_cap.set_mode(SuperCap::BOOST);
    else
        super_cap.set_mode(SuperCap::OFF);

    chassis.update();

    // CAN1 0x200 组帧发送，低速低电流的轮发零电流
    CanFrame frame{.id = 0x200, .size = 8};
    for (M3508& wheel : {std::ref(w1), std::ref(w2), std::ref(w3), std::ref(w4)}) {
        if (std::fabs(wheel.current_ref) < LOW_CURRENT && std::fabs(wheel.speed) < LOW_SPEED)
            wheel.set_current(0);
        wheel.write_can_frame(frame);
    }
    Can(1).send(frame);

    // 超电 100 Hz 发送
    static Timer super_cap_send_timer;
    if (super_cap_send_timer.elapsed() >= 0.01f) {
        super_cap_send_timer.restart();
        // super_cap.send();
    }
}
