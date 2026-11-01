#include "chassis.hpp"
#include "math/math.hpp"

const Chassis::Config& Chassis::config() const {
    return config_;
}

void Chassis::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    set_mode(DETACH_MODE);
    set_motors_enable(is_enable);
}

void Chassis::set_mode(const Mode mode) {
    this->mode = mode;
}

void Chassis::set_speed(const float vx, const float vy, const float wr) {
    this->vx.ref = vx;
    this->vy.ref = vy;
    this->wr.input = wr;
}

void Chassis::set_gimbal_yaw(const Angle gimbal_yaw) {
    gimbal_yaw_ = gimbal_yaw;
}

void Chassis::set_power_limit(const float power_limit) {
    power.limit = power_limit;
}

void Chassis::set_buffer_energy(const float buffer_energy) {
    power.buffer_energy = buffer_energy;
}

void Chassis::update_ui(RobotUi& ui) {
    ui.robot.w1 = wheel_[0]->is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w2 = wheel_[1]->is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w3 = wheel_[2]->is_connect ? RobotUi::GREEN : RobotUi::PINK;
    ui.robot.w4 = wheel_[3]->is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

void Chassis::update() {
    // 离线检测与正解，失能时也更新测量
    update_motors();
    forward();

    // 功率估计
    power.estimate = 0;
    for (const Motor* wheel : wheel_) {
        if (wheel->is_connect)
            power.estimate += wheel->torque * wheel->speed + copper_loss(wheel->current);
    }

    // 失能时清空 PID，电机已失能，发送零命令
    if (!is_enable) {
        clear_pid();
        return;
    }

    // 旋转：用户输入优先，否则按模式分离或跟随
    if (wr.input != 0)
        wr.ref = wr.input;
    else if (mode == FOLLOW_MODE)
        wr.ref = follow_pid_.calculate(gimbal_yaw_);
    else
        wr.ref = 0;
    vz.ref = wr.ref * config_.chassis_radius;

    control();
}

Chassis::Chassis(const Config& config, const std::array<Motor*, 4>& wheel)
    : config_(config), wheel_(wheel), follow_pid_(config.follow_pid_config) {}

void Chassis::set_motors_enable(const bool is_enable) {
    for (Motor* wheel : wheel_)
        wheel->set_enable(is_enable);
}

void Chassis::update_motors() {
    for (Motor* wheel : wheel_)
        wheel->update();
}

bool Chassis::can_drive(const size_t i) const {
    return wheel_[i]->is_connect;
}

void Chassis::clear_pid() {
    follow_pid_.clear();
}

float Chassis::wheel_speed(const size_t i) const {
    return wheel_[i]->is_connect ? wheel_[i]->speed * config_.wheel_radius : 0.0f;
}

float Chassis::wheel_force(const size_t i) const {
    return wheel_[i]->is_connect ? wheel_[i]->torque / config_.wheel_radius : 0.0f;
}

float Chassis::copper_loss(const float current) const {
    // C620 反馈的 iq 与 VESC 同为等幅值变换（最大 20 A），三相铜耗 1.5·R相·iq²
    // 星形接法 R相 = R线/2，即 0.75·R线·iq²
    return 0.75f * config_.wheel_resistance * current * current;
}

std::tuple<float, float, float> Chassis::speed_ref() const {
    const auto [vx, vy] = rotate(this->vx.ref, this->vy.ref, gimbal_yaw_);
    return {vx, vy, vz.ref};
}

void Chassis::set_speed_measure(const float vx, const float vy, const float vz) {
    std::tie(this->vx.measure, this->vy.measure) = rotate(vx, vy, -gimbal_yaw_);
    this->vz.measure = vz;
    wr.measure = vz / config_.chassis_radius;
}
