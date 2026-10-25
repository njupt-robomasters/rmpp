#include "shooter.hpp"

Shooter::Shooter(const Config& config)
    : config(config), rub_pid_(config.rub_pid_config), shoot_pid_(config.shoot_pid_config) {}

void Shooter::set_enable(bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
        rub_pid_.clear();
        shoot_pid_.clear();
    }

    // 这里设置电机
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
    is_heat_protect = heat_remain_ < config.heat_protect;
    // 正逆解
    forward();
    backward();

    // 这里更新电机
}
