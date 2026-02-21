#include "Shooter_42mm.hpp"

Shooter_42mm::Shooter_42mm(const config_t& config, const motor_t& motor) : Shooter(config), motor(motor) {}

void Shooter_42mm::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
    }

    motor.rub_left1.SetEnable(is_enable);
    motor.rub_right1.SetEnable(is_enable);
    motor.rub_left2.SetEnable(is_enable);
    motor.rub_right2.SetEnable(is_enable);
    motor.shoot.SetEnable(is_enable);
}

void Shooter_42mm::HandleUI(UI& ui) {
    if (motor.rub_left1.is_connect == false) {
        ui.robot.rub_left1 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub_left1_measure) < 1) {
        ui.robot.rub_left1 = UI::GREEN;
    } else {
        ui.robot.rub_left1 = UI::ORANGE;
    }

    if (motor.rub_right1.is_connect == false) {
        ui.robot.rub_right1 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub_right1_measure) < 1) {
        ui.robot.rub_right1 = UI::GREEN;
    } else {
        ui.robot.rub_right1 = UI::ORANGE;
    }

    if (motor.rub_left2.is_connect == false) {
        ui.robot.rub_left2 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub_left2_measure) < 1) {
        ui.robot.rub_left2 = UI::GREEN;
    } else {
        ui.robot.rub_left2 = UI::ORANGE;
    }

    if (motor.rub_right2.is_connect == false) {
        ui.robot.rub_right2 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub_right2_measure) < 1) {
        ui.robot.rub_right2 = UI::GREEN;
    } else {
        ui.robot.rub_right2 = UI::ORANGE;
    }

    ui.robot.shoot = motor.shoot.is_connect ? UI::GREEN : UI::PINK;
}

void Shooter_42mm::OnLoop() {
    Shooter::OnLoop();

    // 更新电机
    motor.rub_left1.OnLoop();
    motor.rub_right1.OnLoop();
    motor.rub_left2.OnLoop();
    motor.rub_right2.OnLoop();
    motor.shoot.OnLoop();
}

void Shooter_42mm::backward() {
    // 摩擦轮
    if (is_rub) {
        const UnitFloat<rpm> speed = bullet_speed.ref / config.rub_radius;
        motor.rub_left1.SetSpeed(speed);
        motor.rub_right1.SetSpeed(speed);
        motor.rub_left2.SetSpeed(speed);
        motor.rub_right2.SetSpeed(speed);
    } else {
        motor.rub_left1.SetSpeed(0 * default_unit);
        motor.rub_right1.SetSpeed(0 * default_unit);
        motor.rub_left2.SetSpeed(0 * default_unit);
        motor.rub_right2.SetSpeed(0 * default_unit);
    }

    // 拨弹电机
    if (is_rub && is_shoot) {
        const UnitFloat<rpm> shoot_speed = bullet_freq.ref / config.bullet_per_rev;
        motor.shoot.SetSpeed(shoot_speed);
    } else {
        motor.shoot.SetSpeed(0 * default_unit);
    }
}

void Shooter_42mm::forward() {
    // 摩擦轮
    rub_left1_measure = motor.rub_left1.speed.measure * config.rub_radius;
    rub_right1_measure= motor.rub_right1.speed.measure * config.rub_radius;
    rub_left2_measure = motor.rub_left2.speed.measure * config.rub_radius;
    rub_right2_measure = motor.rub_right2.speed.measure * config.rub_radius;
    bullet_speed.measure = (rub_left2_measure + rub_right2_measure) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_rev;

    // 拨弹电机电流
    shoot_current = motor.shoot.current.measure;
}