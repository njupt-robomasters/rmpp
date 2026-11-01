#include "shooter.hpp"
#include <cmath>

Shooter::Shooter(const Config& config)
    : config(config), shoot_pid_(config.shoot_pid_config) {}

void Shooter::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态，摩擦轮 PID 在 backward 中随 is_rub 清空
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
        shoot_pid_.clear();
    }

    set_motors_enable(is_enable);
}

void Shooter::set_bullet_speed(const float bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter::set_bullet_freq(const float bullet_freq) {
    this->bullet_freq.ref = bullet_freq;
}

void Shooter::set_rub(const bool is_rub) {
    this->is_rub = is_rub;
}

void Shooter::set_shoot(const bool is_shoot) {
    this->is_shoot = is_shoot;
}

void Shooter::set_heat_remain(int32_t heat_remain) {
    heat_remain_ = heat_remain;
    is_heat_protect = heat_remain_ < config.heat_protect;
}

void Shooter::update() {
    update_motors();
    is_heat_protect = heat_remain_ < config.heat_protect;

    // 正逆解
    forward();
    backward();
}

void Shooter::control_rub(Motor& motor, Pid& pid, const float speed_ref, const float speed_measure) {
    if (is_rub && motor.is_connect) {
        motor.set_enable(true);
        set_output(motor, config.rub_output, pid.calculate(speed_ref - speed_measure));
        return;
    }

    pid.clear();
    motor.set_current(0);
    if (!is_rub && is_enable && config.auto_release_speed > 0)
        motor.set_enable(std::fabs(motor.speed) >= config.auto_release_speed);
}

void Shooter::set_output(Motor& motor, const PidOutput type, const float output) {
    if (type == CURRENT_OUTPUT)
        motor.set_current(output);
    else
        motor.set_torque(output);
}
