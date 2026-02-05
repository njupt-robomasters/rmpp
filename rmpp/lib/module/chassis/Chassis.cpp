#include "Chassis.hpp"

Chassis::Chassis(const config_t& config) : config(config),
                                           follow_pid(config.follow_pid_config) {}

void Chassis::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    // 这里设置电机
}

void Chassis::SetMode(mode_e mode) {
    this->mode = mode;
}

void Chassis::SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr; // 这里是角速度，在calcPID中会计算底盘跟随，同时补充ref和vz
}

void Chassis::SetGimbalYaw(const Angle<>& gimbal_yaw) {
    this->gimbal_yaw = gimbal_yaw;
}

void Chassis::SetPowerLimit(const UnitFloat<>& power_limit) {
    power.limit = power_limit;
}

void Chassis::SetBufferEnergy(const UnitFloat<>& buffer_energy) {
    power.buffer_energy = buffer_energy;
}

void Chassis::OnLoop() {
    handleFollow();

    forward();
    backward();

    // 这里更新电机
}

void Chassis::handleFollow() {
    if (wr.input != 0) { // 优先用户旋转输入
        wr.ref = wr.input;
    } else {
        if (mode == DETACH_MODE) { // 底盘分离模式
            wr.ref = 0 * default_unit;
        } else if (mode == FOLLOW_MODE) { // 底盘跟随模式
            wr.ref = follow_pid.Calculate(gimbal_yaw);
        }
    }
    vz.ref = wr.ref * config.chassis_radius; // 计算线速度
}