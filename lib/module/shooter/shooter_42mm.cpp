#include "shooter_42mm.hpp"
#include "math/angle.hpp"
#include "math/conversions.hpp"
#include <cmath>

using math::Angle;

Shooter42mm::Shooter42mm(const Config& config, const Motors& motor) : Shooter(config), motor(motor) {}

void Shooter42mm::set_enable(const bool is_enable) {
    if (this->is_enable == is_enable)
        return;
    this->is_enable = is_enable;

    // 失能复位摩擦轮和拨弹电机状态
    if (!is_enable) {
        is_rub = false;
        is_shoot = false;
    }

    motor.rub1.set_enable(is_enable);
    motor.rub2.set_enable(is_enable);
    motor.rub3.set_enable(is_enable);
    motor.rub4.set_enable(is_enable);
    motor.shoot.set_enable(is_enable);
}

void Shooter42mm::update_ui(Ui& ui) {
    if (motor.rub1.is_connect == false) {
        ui.robot.rub1 = Ui::PINK;
    } else if (std::fabsf(bullet_speed.ref * RUB12_RATIO - rub1_measure_) < 1) {
        ui.robot.rub1 = Ui::GREEN;
    } else {
        ui.robot.rub1 = Ui::ORANGE;
    }

    if (motor.rub2.is_connect == false) {
        ui.robot.rub2 = Ui::PINK;
    } else if (std::fabsf(bullet_speed.ref * RUB12_RATIO - rub2_measure_) < 1) {
        ui.robot.rub2 = Ui::GREEN;
    } else {
        ui.robot.rub2 = Ui::ORANGE;
    }

    if (motor.rub3.is_connect == false) {
        ui.robot.rub3 = Ui::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub3_measure_) < 1) {
        ui.robot.rub3 = Ui::GREEN;
    } else {
        ui.robot.rub3 = Ui::ORANGE;
    }

    if (motor.rub4.is_connect == false) {
        ui.robot.rub4 = Ui::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub4_measure_) < 1) {
        ui.robot.rub4 = Ui::GREEN;
    } else {
        ui.robot.rub4 = Ui::ORANGE;
    }

    ui.robot.shoot = motor.shoot.is_connect ? Ui::GREEN : Ui::PINK;
}

void Shooter42mm::on_loop() {
    Shooter::on_loop();

    // 更新电机
    motor.rub1.on_loop();
    motor.rub2.on_loop();
    motor.rub3.on_loop();
    motor.rub4.on_loop();
    motor.shoot.on_loop();
}

void Shooter42mm::backward() {
    // 摩擦轮
    if (is_rub) {
        const float speed = bullet_speed.ref / config.rub_radius;
        motor.rub1.set_speed(speed * RUB12_RATIO);
        motor.rub2.set_speed(speed * RUB12_RATIO);
        motor.rub3.set_speed(speed);
        motor.rub4.set_speed(speed);
    } else {
        motor.rub1.set_speed(0.0f);
        motor.rub2.set_speed(0.0f);
        motor.rub3.set_speed(0.0f);
        motor.rub4.set_speed(0.0f);
    }

    // 拨弹电机
    if (!is_rub) { // 关摩擦轮
        state_ = FREE;
    }

    switch (state_) {
    case FREE: {
        motor.shoot.set_angle(motor.shoot.angle.measure); // 保持当前角度

        if (is_rub) { // 开摩擦轮
            state_ = SHAKE;
            timer_shake_.update_dt();
            shake_upper_ = motor.shoot.angle.measure;
        }
    } break;

    case SHAKE: {
        const Angle angle_per_bullet = 1 / config.bullet_per_angle;
        const float a = angle_per_bullet * SHANE_AMP / 2;
        const float offset = shake_upper_ - a;
        const float t = timer_shake_.get_dt();
        const Angle angle = a * std::cosf(2 * math::pi * SHAKE_FREQ * t) + offset;
        motor.shoot.set_angle(angle);

        // 防止计时器溢出
        if (SHAKE_FREQ * t > 1) {
            timer_shake_.update_dt();
        }

        if (is_shoot && !is_shoot_last_ && !is_heat_protect) { // 发射信号上升沿，热量保护
            // 增加角度
            const Angle angle_per_bullet = 1 / config.bullet_per_angle;
            Angle target_angle = shake_upper_ + angle_per_bullet;

            // 对齐相位
            const int8_t index = (int8_t)std::round((target_angle / angle_per_bullet)); // 四舍五入
            target_angle = index * angle_per_bullet;

            // 设置电机角度
            motor.shoot.set_angle(target_angle);

            state_ = SHOOT;
            timer_block_.update_dt();
        }
    } break;

    case SHOOT: {
        // 拨弹计时
        const Angle angle_per_bullet = 1 / config.bullet_per_angle;
        if (std::fabsf(motor.shoot.angle.ref - motor.shoot.angle.measure) < (1.0f - SHOOT_FINISH_RATIO) * angle_per_bullet) {
            state_ = SHAKE;
            timer_shake_.update_dt();
            shake_upper_ = motor.shoot.angle.ref;
        }

        // 卡弹检测
        if (motor.shoot.torque.ref >= motor.shoot.config.angle_pid_config->max_out) { // 达到角度环最大输出
            if (timer_block_.get_dt() > config.block_time) {                          // 卡弹计时
                state_ = SHAKE;
                timer_shake_.update_dt();
                shake_upper_ = motor.shoot.angle.measure;
            }
        } else {
            timer_block_.update_dt(); // 未卡弹，重置计时器
        }
    } break;

    default:
        break;
    }
    is_shoot_last_ = is_shoot;
}

void Shooter42mm::forward() {
    // 摩擦轮
    rub1_measure_ = motor.rub1.speed.measure * config.rub_radius;
    rub2_measure_ = motor.rub2.speed.measure * config.rub_radius;
    rub3_measure_ = motor.rub3.speed.measure * config.rub_radius;
    rub4_measure_ = motor.rub4.speed.measure * config.rub_radius;
    bullet_speed.measure = (rub3_measure_ + rub4_measure_) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_angle;

    // 拨弹电机电流
    shoot_current = motor.shoot.current.measure;
}