#include "shooter.hpp"

Shooter::Shooter(PID::param_t* m2006_pid_param) :
    m2006(1, 0x206),
    n630_1(2, 96),
    n630_2(2, 98) {
    m2006.SetInvert(true),
    m2006.SetPIDParam(m2006_pid_param);
    SetBulletSpeed(24.0f);
    SetShootFreq(5.0f);
}

void Shooter::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 设置子设备
    m2006.SetEnable(is_enable);
    n630_1.SetEnable(is_enable);
    n630_2.SetEnable(is_enable);
}

void Shooter::SetBulletSpeed(const Unit<m_s> bullet_speed) {
    this->bullet_speed = bullet_speed;
    n630_speed = bullet_speed / WHEEL_RADIUS * 7;
    n630_1.SetSpeed(n630_speed);
    n630_2.SetSpeed(n630_speed);
}

void Shooter::SetShootFreq(const Unit<Hz> shoot_freq) {
    this->shoot_freq.ref = shoot_freq;
    m2006.SetSpeed(shoot_freq / SHOOT_PRE_ROUND * rps);
}

void Shooter::SetPrepareShoot(const bool is_prepare_shoot) {
    this->is_prepare_shoot = is_prepare_shoot;
    n630_1.SetEnable(is_prepare_shoot);
    n630_2.SetEnable(is_prepare_shoot);
}

void Shooter::SetShoot(const bool is_shoot) {
    this->is_shoot = is_shoot;
    m2006.SetEnable(is_shoot);
}


void Shooter::Update() {
    shoot_freq.measure = m2006.speed.measure.get(rps) * SHOOT_PRE_ROUND;

    m2006.Update();
    n630_1.Update();
    n630_2.Update();
}
