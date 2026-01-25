#include "Shooter_17mm.hpp"

Shooter_17mm::Shooter_17mm(const config_t& config, const motor_t& motor) : Shooter(config), motor(motor) {}

void Shooter_17mm::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
    }

    motor.rub_left.SetEnable(is_enable);
    motor.rub_right.SetEnable(is_enable);
    motor.shoot.SetEnable(is_enable);
}

void Shooter_17mm::OnLoop() {
    // 速度解算
    speedBackward();
    speedForward();

    // 更新电机
    motor.rub_left.OnLoop();
    motor.rub_right.OnLoop();
    motor.shoot.OnLoop();
}

void Shooter_17mm::speedForward() {
    // 摩擦轮
    if (is_rub) {
        const UnitFloat<rpm> n630_speed = bullet_speed.ref / config.rub_radius;
        motor.rub_left.SetSpeed(n630_speed);
        motor.rub_right.SetSpeed(n630_speed);
    } else {
        motor.rub_left.SetSpeed(0 * default_unit);
        motor.rub_right.SetSpeed(0 * default_unit);
    }

    // 拨弹电机
    if (is_rub && is_shoot) {
        const UnitFloat<rpm> shoot_speed = bullet_freq.ref / config.bullet_per_rev;
        motor.shoot.SetSpeed(shoot_speed);
    } else {
        motor.shoot.SetSpeed(0 * default_unit);
    }
}

void Shooter_17mm::speedBackward() {
    // 摩擦轮
    bullet_speed.measure.left = motor.rub_left.speed.measure * config.rub_radius;
    bullet_speed.measure.right = motor.rub_right.speed.measure * config.rub_radius;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_rev;
}