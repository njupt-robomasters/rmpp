#include "shooter.hpp"

Shooter::Shooter(const Config& config) : config(config) {}

void Shooter::set_enable(bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
    }

    // 这里设置电机
}

void Shooter::set_bullet_speed(float bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter::set_bullet_freq(float bullet_freq) {
    this->bullet_freq.ref = bullet_freq;
}

void Shooter::set_rub(const bool is_rub) {
    this->is_rub = is_rub;
}

void Shooter::set_shoot(const bool is_shoot) {
    this->is_shoot = is_shoot;
}

void Shooter::set_heat_protect(bool is_heat_protect) {
    this->is_heat_protect = is_heat_protect;
}

void Shooter::on_loop() {
    // 正逆解
    forward();
    backward();

    // 这里更新电机
}