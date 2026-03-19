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

    motor.rub1.SetEnable(is_enable);
    motor.rub2.SetEnable(is_enable);
    motor.rub3.SetEnable(is_enable);
    motor.rub4.SetEnable(is_enable);
    motor.shoot.SetEnable(is_enable);
}

void Shooter_42mm::UpdateUI(UI& ui) {
    if (motor.rub1.is_connect == false) {
        ui.robot.rub1 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref * RUB12_RATIO - rub1_measure) < 1) {
        ui.robot.rub1 = UI::GREEN;
    } else {
        ui.robot.rub1 = UI::ORANGE;
    }

    if (motor.rub2.is_connect == false) {
        ui.robot.rub2 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref * RUB12_RATIO - rub2_measure) < 1) {
        ui.robot.rub2 = UI::GREEN;
    } else {
        ui.robot.rub2 = UI::ORANGE;
    }

    if (motor.rub3.is_connect == false) {
        ui.robot.rub3 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub3_measure) < 1) {
        ui.robot.rub3 = UI::GREEN;
    } else {
        ui.robot.rub3 = UI::ORANGE;
    }

    if (motor.rub4.is_connect == false) {
        ui.robot.rub4 = UI::PINK;
    } else if (unit::abs(bullet_speed.ref - rub4_measure) < 1) {
        ui.robot.rub4 = UI::GREEN;
    } else {
        ui.robot.rub4 = UI::ORANGE;
    }

    ui.robot.shoot = motor.shoot.is_connect ? UI::GREEN : UI::PINK;
}

void Shooter_42mm::OnLoop() {
    Shooter::OnLoop();

    // 更新电机
    motor.rub1.OnLoop();
    motor.rub2.OnLoop();
    motor.rub3.OnLoop();
    motor.rub4.OnLoop();
    motor.shoot.OnLoop();
}

void Shooter_42mm::backward() {
    // 摩擦轮
    if (is_rub) {
        const UnitFloat<rpm> speed = bullet_speed.ref / config.rub_radius;
        motor.rub1.SetSpeed(speed * RUB12_RATIO);
        motor.rub2.SetSpeed(speed * RUB12_RATIO);
        motor.rub3.SetSpeed(speed);
        motor.rub4.SetSpeed(speed);
    } else {
        motor.rub1.SetSpeed(0 * default_unit);
        motor.rub2.SetSpeed(0 * default_unit);
        motor.rub3.SetSpeed(0 * default_unit);
        motor.rub4.SetSpeed(0 * default_unit);
    }

    // 拨弹电机
    if (!is_rub) { // 关摩擦轮
        state = FREE;
    }

    switch (state) {
        case FREE: {
            motor.shoot.SetAngle(motor.shoot.angle.measure); // 保持当前角度

            if (is_rub) { // 开摩擦轮
                state = SHAKE;
                dwt_shake.UpdateDT();
                shake_upper = motor.shoot.angle.measure;
            }
        }
        break;

        case SHAKE: {
            const Angle<deg> angle_per_bullet = 1 / config.bullet_per_angle;
            const UnitFloat<deg> a = angle_per_bullet * SHANE_AMP / 2;
            const UnitFloat<deg> offset = shake_upper - a;
            const UnitFloat<s> t = dwt_shake.GetDT();
            const Angle<deg> angle = a * cos(2 * PI * SHAKE_FREQ * t) + offset;
            motor.shoot.SetAngle(angle);

            // 防止计时器溢出
            if (SHAKE_FREQ * t > 1) {
                dwt_shake.UpdateDT();
            }

            if (is_shoot && !is_shoot_last && !is_heat_protect) { // 发射信号上升沿，热量保护
                // 增加角度
                const Angle<deg> angle_per_bullet = 1 / config.bullet_per_angle;
                Angle<deg> target_angle = shake_upper + angle_per_bullet;

                // 对齐相位
                const int8_t index = (int8_t)std::round((target_angle / angle_per_bullet).toFloat()); // 四舍五入
                target_angle = index * angle_per_bullet;

                // 设置电机角度
                motor.shoot.SetAngle(target_angle);

                state = SHOOT;
                dwt_block.UpdateDT();
            }
        }
        break;

        case SHOOT: {
            // 拨弹计时
            if (abs(motor.shoot.angle.ref - motor.shoot.angle.measure) < SHOOT_FINISH_ERR) {
                state = SHAKE;
                dwt_shake.UpdateDT();
                shake_upper = motor.shoot.angle.ref;
            }

            // 卡弹检测
            if (motor.shoot.torque.ref >= motor.shoot.config.angle_pid_config->max_out) { // 达到角度环最大输出
                if (dwt_block.GetDT() > config.block_time) {                              // 卡弹计时
                    state = SHAKE;
                    dwt_shake.UpdateDT();
                    shake_upper = motor.shoot.angle.measure;
                }
            } else {
                dwt_block.UpdateDT(); // 未卡弹，重置计时器
            }
        }
        break;

        default:
            break;
    }
    is_shoot_last = is_shoot;
}

void Shooter_42mm::forward() {
    // 摩擦轮
    rub1_measure = motor.rub1.speed.measure * config.rub_radius;
    rub2_measure = motor.rub2.speed.measure * config.rub_radius;
    rub3_measure = motor.rub3.speed.measure * config.rub_radius;
    rub4_measure = motor.rub4.speed.measure * config.rub_radius;
    bullet_speed.measure = (rub3_measure + rub4_measure) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_angle;

    // 拨弹电机电流
    shoot_current = motor.shoot.current.measure;
}