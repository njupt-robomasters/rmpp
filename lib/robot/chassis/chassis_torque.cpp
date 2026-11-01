#include "chassis_torque.hpp"
#include "math/math.hpp"
#include <cmath>

ChassisTorque::ChassisTorque(const Config& config, const Pid::Config* vxyz_pid_config, const std::array<Motor*, 4>& wheel)
    : Chassis(config, wheel), vx_pid_(vxyz_pid_config), vy_pid_(vxyz_pid_config), vz_pid_(vxyz_pid_config) {}

void ChassisTorque::clear_pid() {
    Chassis::clear_pid();
    vx_pid_.clear();
    vy_pid_.clear();
    vz_pid_.clear();
}

void ChassisTorque::control() {
    // 速度 PID
    fx.ref = vx_pid_.calculate(vx.ref - vx.measure);
    fy.ref = vy_pid_.calculate(vy.ref - vy.measure);
    fz.ref = vz_pid_.calculate(vz.ref - vz.measure);

    power_control();
    backward();

    // 不能驱动的轮输出零
    for (size_t i = 0; i < wheel_.size(); i++) {
        if (!can_drive(i))
            wheel_[i]->set_torque(0);
    }
}

std::tuple<float, float, float> ChassisTorque::force_ref() const {
    const auto [fx, fy] = rotate(this->fx.ref, this->fy.ref, gimbal_yaw_);
    return {fx, fy, fz.ref};
}

void ChassisTorque::set_force_measure(const float fx, const float fy, const float fz) {
    std::tie(this->fx.measure, this->fy.measure) = rotate(fx, fy, -gimbal_yaw_);
    this->fz.measure = fz;
}

void ChassisTorque::power_control() {
    gain_xy = gain_z = 1;

    // 裁判系统不限功率时读出 65535，此时缓冲能量也异常，必须跳过
    if (power.limit == 65535)
        return;

    // 基于功率：旋转超过保留速度时优先衰减旋转
    const bool is_spin = std::fabsf(wr.measure) > config_.reserve_wr;
    const float power_xy = fx.ref * vx.measure + fy.ref * vy.measure;
    const float power_z = fz.ref * vz.measure;
    if (is_spin) {
        if (power_xy > power.limit) { // 停止旋转，衰减移动
            gain_xy *= power.limit / power_xy;
            gain_z = 0;
        } else if (power_xy + power_z > power.limit) { // 衰减旋转
            gain_z *= (power.limit - power_xy) / power_z;
        }
    } else if (power_xy + power_z > power.limit) { // 同时衰减
        gain_xy *= power.limit / (power_xy + power_z);
        gain_z *= power.limit / (power_xy + power_z);
    }

    // 基于缓冲能量：低于 30 J 停止，旋转时 45 J 以下先停旋转
    const float buffer_energy = power.buffer_energy;
    if (buffer_energy < 30) {
        gain_xy = gain_z = 0;
    } else if (!is_spin) {
        gain_xy *= (buffer_energy - 30) / 30;
        gain_z *= (buffer_energy - 30) / 30;
    } else if (buffer_energy < 45) {
        gain_xy *= (buffer_energy - 30) / 15;
        gain_z = 0;
    } else {
        gain_z *= (buffer_energy - 45) / 15;
    }

    // 衰减牵引力
    fx.ref *= gain_xy;
    fy.ref *= gain_xy;
    fz.ref *= gain_z;
}
