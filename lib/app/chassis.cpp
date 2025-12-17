#include "chassis.hpp"

void Chassis_Template::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;
    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    // 这里设置电机
}

void Chassis_Template::SetMode(mode_e mode) {
    this->mode = mode;
}

void Chassis_Template::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr) {
    this->vx.gimbal.ref = vx;
    this->vy.gimbal.ref = vy;
    this->vr.ref.input = vr;
}

void Chassis_Template::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis_Template::SetPowerLimit(const UnitFloat<>& power) {
    this->power_limit = power;
}

void Chassis_Template::calcFollow() {
    if (mode == DETACH_MODE || vr.ref.input != 0) { // 小陀螺状态下强制分离模式
        vr.ref.follow = 0 * default_unit;
    } else if (mode == FOLLOW_MODE) {
        vr.ref.follow = follow_pid.Calculate(gimbal_yaw);
    }
    vr.ref.sum = vr.ref.input + vr.ref.follow;
}

void Chassis_Template::OnLoop() {
    // 底盘跟随
    calcFollow();

    // 运动学解算
    speedBackward();
    speedForward();

    // 这里更新电机

    // 功率控制
    powerControl();
}