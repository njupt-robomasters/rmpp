#include "shooter.hpp"

void Shooter_Template::SetEnable(bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_prepare_shoot = false;
        is_shoot = false;
    }

    // 这里设置电机
}

void Shooter_Template::SetBulletSpeed(const UnitFloat<>& bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter_Template::SetBulletFreq(const UnitFloat<>& bullet_freq) {
    this->bullet_freq.ref = bullet_freq;
}

void Shooter_Template::SetPrepareShoot(const bool is_prepare_shoot) {
    this->is_prepare_shoot = is_prepare_shoot;
}

void Shooter_Template::SetShoot(const bool is_shoot) {
    this->is_shoot = is_shoot;
    if (is_shoot) { // 如果开始拨弹，强制开启摩擦轮
        this->is_prepare_shoot = true;
    }
}

void Shooter_Template::OnLoop() {
    // 速度解算
    speedBackward();
    speedForward();

    // 这里更新电机
}