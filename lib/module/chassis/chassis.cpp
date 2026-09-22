#include "chassis.hpp"
#include "math/angle.hpp"
#include <cmath>

Chassis::Chassis(const Config& config) : config(config),
                                         vx_pid_(config.vxyz_pid_config), vy_pid_(config.vxyz_pid_config), vz_pid_(config.vxyz_pid_config),
                                         follow_pid_(config.follow_pid_config) {}

void Chassis::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    set_mode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    // 这里设置电机
}

void Chassis::set_mode(const Mode mode) {
    this->mode = mode;
}

void Chassis::set_speed(float vx, float vy, float wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr; // 这里是角速度，在calcPID中会计算底盘跟随，同时补充ref和vz
}

void Chassis::set_gimbal_yaw(Angle gimbal_yaw) {
    this->gimbal_yaw_ = gimbal_yaw;
}

void Chassis::set_power_limit(float power_limit) {
    power.limit = power_limit;
}

void Chassis::set_buffer_energy(float buffer_energy) {
    power.buffer_energy = buffer_energy;
}

void Chassis::on_loop() {
    // 失能清空PID
    if (is_enable == false) {
        vx_pid_.clear();
        vy_pid_.clear();
        vz_pid_.clear();
        follow_pid_.clear();
        return;
    }

    forward();       // 速度和力学正解
    calc_pid();      // 计算PID
    power_control(); // 功率控制
    backward();      // 速度和力学逆解

    // 这里更新电机
}

void Chassis::calc_pid() {
    // 底盘跟随PID
    if (wr.input != 0) { // 优先用户旋转输入
        wr.ref = wr.input;
    } else {
        if (mode == DETACH_MODE) { // 底盘分离模式
            wr.ref = 0.0f;
        } else if (mode == FOLLOW_MODE) { // 底盘跟随模式
            wr.ref = follow_pid_.calculate(gimbal_yaw_);
        }
    }
    vz.ref = wr.ref * config.chassis_radius; // 计算线速度

    fx.ref = vx_pid_.calculate(vx.ref - vx.measure);
    fy.ref = vy_pid_.calculate(vy.ref - vy.measure);
    fz.ref = vz_pid_.calculate(vz.ref - vz.measure);
}

void Chassis::power_control() {
    // 功率估计
    power.estimate_xy = fx.measure * vx.measure + fy.measure * vy.measure;
    power.estimate_z = fz.measure * vz.measure;
    power.estimate = power.estimate_xy + power.estimate_z;

    // 裁判系统设置无限功率时，读出来是65535，禁用功率控制（这时候缓冲能量还在异常工作，必须在这里禁用）
    if (power.limit == 65535)
        return;

    // 牵引力衰减系数
    auto& gain_x = power.gain_x;
    auto& gain_y = power.gain_y;
    auto& gain_xy = power.gain_xy;
    auto& gain_z = power.gain_z;
    gain_x = gain_y = gain_xy = gain_z = 1.0f; // 初始化为1，方便后面*=运算

    // 基于功率估计的功率控制
    const float power_xy = fx.ref * vx.measure + fy.ref * vy.measure;
    const float power_z = fz.ref * vz.measure;
    if (std::fabsf(wr.measure) > config.reserve_wr) { // 旋转速度 > 保留速度，优先衰减旋转
        if (power_xy > power.limit) {                 // 停止旋转，衰减移动
            gain_xy *= power.limit / power_xy;
            gain_z *= 0;
        } else if (power_xy + power_z > power.limit) { // 衰减旋转
            gain_z *= (power.limit - power_xy) / power_z;
        }
    } else {
        if (power_xy + power_z > power.limit) { // 同时衰减
            gain_xy *= power.limit / (power_xy + power_z);
            gain_z *= power.limit / (power_xy + power_z);
        }
    }

    // 基于缓冲能量的功率控制
    const auto& buffer_energy = power.buffer_energy;
    if (buffer_energy < 30) { // 停止移动和旋转
        gain_xy *= 0;
        gain_z *= 0;
    } else {
        if (std::fabsf(wr.measure) > config.reserve_wr) { // 旋转速度 > 保留速度，优先衰减旋转
            if (buffer_energy < 45) {                     // 停止旋转，衰减移动
                gain_xy *= (buffer_energy - 30) / (15);
                gain_z *= 0;
            } else { // 衰减旋转
                gain_z *= (buffer_energy - 45) / (15);
            }
        } else { // 同时衰减
            gain_xy *= (buffer_energy - 30) / (30);
            gain_z *= (buffer_energy - 30) / (30);
        }
    }

    // 应用牵引力衰减
    fx.ref *= gain_xy;
    fy.ref *= gain_xy;
    fz.ref *= gain_z;
}