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

void Shooter_17mm::HandleUI(UI& ui) {
    if (motor.rub_left.is_connect == false) {
        ui.robot.rub_left1 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub_left_measure) < 1) {
        ui.robot.rub_left1 = UI::GREEN;
    } else {
        ui.robot.rub_left1 = UI::ORANGE;
    }

    if (motor.rub_right.is_connect == false) {
        ui.robot.rub_right1 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub_right_measure) < 1) {
        ui.robot.rub_right1 = UI::GREEN;
    } else {
        ui.robot.rub_right1 = UI::ORANGE;
    }

    ui.robot.shoot = motor.shoot.is_connect ? UI::GREEN : UI::PINK;
}

void Shooter_17mm::OnLoop() {
    Shooter::OnLoop();

    // 更新电机
    motor.rub_left.OnLoop();
    motor.rub_right.OnLoop();
    motor.shoot.OnLoop();
}

void Shooter_17mm::backward() {
    // 摩擦轮
    if (is_rub) {
        const UnitFloat<rpm> speed = bullet_speed.ref / config.rub_radius;
        motor.rub_left.SetSpeed(speed);
        motor.rub_right.SetSpeed(speed);
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

void Shooter_17mm::forward() {
    // 摩擦轮
    rub_left_measure = motor.rub_left.speed.measure * config.rub_radius;
    rub_right_measure = motor.rub_right.speed.measure * config.rub_radius;
    bullet_speed.measure = (rub_left_measure + rub_right_measure) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_rev;

    // 拨弹电机电流
    shoot_current = motor.shoot.current.measure;
}