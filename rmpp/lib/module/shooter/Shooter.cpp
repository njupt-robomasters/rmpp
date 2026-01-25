#include "Shooter.hpp"

Shooter::Shooter(const config_t& config) : config(config) {}

void Shooter::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
    }

    // 这里设置电机
}

void Shooter::SetBulletSpeed(const UnitFloat<>& bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter::SetBulletFreq(const UnitFloat<>& bullet_freq) {
    this->bullet_freq.ref = bullet_freq;
}

void Shooter::SetRub(const bool is_rub) {
    this->is_rub = is_rub;
}

void Shooter::SetShoot(const bool is_shoot) {
    this->is_shoot = is_shoot;
}

void Shooter::OnLoop() {
    // 速度解算
    speedBackward();
    speedForward();

    // 这里更新电机
}