#include "Chassis.hpp"

Chassis::Chassis(const config_t& config) : config(config), follow_pid(config.follow_pid_config) {}

void Chassis::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    // 这里设置电机
}

void Chassis::SetMode(mode_e mode) {
    this->mode = mode;
}

void Chassis::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr) {
    this->vx.gimbal.ref = vx;
    this->vy.gimbal.ref = vy;
    this->wr.input = vr;
}

void Chassis::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis::SetPowerLimit(const UnitFloat<>& power) {
    this->power.limit = power;
}

void Chassis::SetBufferEnergy(const UnitFloat<>& buffer_energy) {
    this->power.buffer_energy = buffer_energy;
}

void Chassis::calcFollow() {
    if (mode == DETACH_MODE || wr.ref != 0) { // 小陀螺状态下强制分离模式
        wr.follow = 0 * default_unit;
    } else if (mode == FOLLOW_MODE) {
        wr.follow = follow_pid.Calculate(gimbal_yaw);
    }
    wr.ref = wr.input + wr.follow;
}

void Chassis::OnLoop() {
    // 底盘跟随
    calcFollow();

    // 运动学解算
    speedBackward();
    speedForward();

    // 这里计算电机PID

    // 功率控制
    powerControl();
}