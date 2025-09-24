#include "shooter.hpp"

Shooter::Shooter(PID::param_t* shoot_pid_param) :
    m_shoot(1, 0x206),
    n630_1(2, 96),
    n630_2(2, 98) {
    // 设置电机反转
    m_shoot.SetInvert(true),
        // 设置电机PID参数
        m_shoot.SetPIDParam(shoot_pid_param);
    // 设置默认弹速
    SetBulletSpeed(24.0f);
    // 设置默认弹频
    SetShootFreq(1.0f);
}

void Shooter::forwardCalc() {
    if (is_prepare_shoot) {
        const UnitFloat<rpm> n630_speed = bullet_speed.ref / WHEEL_RADIUS;
        n630_1.SetSpeed(n630_speed);
        n630_2.SetSpeed(n630_speed);
    } else {
        n630_1.SetSpeed(0);
        n630_2.SetSpeed(0);
    }

    if (is_shoot) {
        m_shoot.SetSpeed(shoot_freq.ref * rps / SHOOT_PRE_ROUND);
    } else {
        m_shoot.SetSpeed(0);
    }
}

void Shooter::backwardCalc() {
    bullet_speed.measure1 = n630_1.speed.measure * WHEEL_RADIUS;
    bullet_speed.measure2 = n630_2.speed.measure * WHEEL_RADIUS;
    shoot_freq.measure = m_shoot.speed.measure.get(rps) * SHOOT_PRE_ROUND;
}

void Shooter::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetPrepareShoot(false);
    SetShoot(false);

    m_shoot.SetEnable(is_enable);
    n630_1.SetEnable(is_enable);
    n630_2.SetEnable(is_enable);
}

void Shooter::SetBulletSpeed(const UnitFloat<>& bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter::SetShootFreq(const UnitFloat<>& shoot_freq) {
    this->shoot_freq.ref = shoot_freq;
}

void Shooter::SetPrepareShoot(const bool is_prepare_shoot) {
    this->is_prepare_shoot = is_prepare_shoot;
}

void Shooter::SetShoot(const bool is_shoot) {
    this->is_shoot = is_shoot;
}

void Shooter::Update() {
    backwardCalc();
    forwardCalc();

    m_shoot.Update();
    n630_1.Update();
    n630_2.Update();
}
