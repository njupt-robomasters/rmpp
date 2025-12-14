#include "shooter.hpp"

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
