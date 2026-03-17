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

    motor.rub1.SetEnable(is_enable);
    motor.rub2.SetEnable(is_enable);
    motor.shoot.SetEnable(is_enable);
}

void Shooter_17mm::UpdateUI(UI& ui) {
    if (motor.rub1.is_connect == false) {
        ui.robot.rub1 = UI::PINK;
        ui.robot.rub3 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub1_measure) > 1) {
        ui.robot.rub1 = UI::ORANGE;
        ui.robot.rub3 = UI::ORANGE;
    } else {
        ui.robot.rub1 = UI::GREEN;
        ui.robot.rub3 = UI::GREEN;
    }

    if (motor.rub2.is_connect == false) {
        ui.robot.rub2 = UI::PINK;
        ui.robot.rub4 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub2_measure) > 1) {
        ui.robot.rub2 = UI::ORANGE;
        ui.robot.rub4 = UI::ORANGE;
    } else {
        ui.robot.rub2 = UI::GREEN;
        ui.robot.rub4 = UI::GREEN;
    }

    ui.robot.shoot = motor.shoot.is_connect ? UI::GREEN : UI::PINK;
}

void Shooter_17mm::OnLoop() {
    Shooter::OnLoop();

    // 更新电机
    motor.rub1.OnLoop();
    motor.rub2.OnLoop();
    motor.shoot.OnLoop();
}

void Shooter_17mm::backward() {
    // 摩擦轮
    if (is_rub) {
        const UnitFloat<rpm> speed = bullet_speed.ref / config.rub_radius;
        motor.rub1.SetSpeed(speed);
        motor.rub2.SetSpeed(speed);
    } else {
        motor.rub1.SetSpeed(0 * default_unit);
        motor.rub2.SetSpeed(0 * default_unit);
    }

    // 拨弹电机
    if (is_rub && !is_heat_protect && is_shoot) { // 确保摩擦轮开启、枪口热量保护、发射信号
        if (!is_block) { // 未卡弹
            // 正转
            const UnitFloat<rpm> shoot_speed = bullet_freq.ref / config.bullet_per_angle;
            motor.shoot.SetSpeed(shoot_speed);

            // 卡弹检测
            if (motor.shoot.current.ref >= motor.shoot.config.speed_pid_config->max_out) { // 达到速度环最大输出
                // 卡弹计时
                if (dwt_block.GetDT() > config.block_time) {
                    is_block = true;
                    dwt_block.UpdateDT(); // 重置计时器，用于倒转计时
                }
            } else {
                dwt_block.UpdateDT(); // 未卡弹，重置计时器
            }
        } else { // 卡弹
            // 反转
            const UnitFloat<rpm> shoot_speed = bullet_freq.ref / config.bullet_per_angle;
            motor.shoot.SetSpeed(-shoot_speed);

            // 反转计时
            if (dwt_block.GetDT() > config.reverse_time) {
                is_block = false;
                dwt_block.UpdateDT(); // 重置计时器，用于卡弹计时
            }
        }
    } else {
        is_block = false;                       // 清除卡弹状态
        dwt_block.UpdateDT();                   // 重置计时器
        motor.shoot.SetSpeed(0 * default_unit); // 速度置零
    }
}

void Shooter_17mm::forward() {
    // 摩擦轮
    rub1_measure = motor.rub1.speed.measure * config.rub_radius;
    rub2_measure = motor.rub2.speed.measure * config.rub_radius;
    bullet_speed.measure = (rub1_measure + rub2_measure) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_angle;

    // 拨弹电机电流
    shoot_current = motor.shoot.current.measure;
}