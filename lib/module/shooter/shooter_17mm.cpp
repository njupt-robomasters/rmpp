#include "shooter_17mm.hpp"
#include <cmath>

Shooter17mm::Shooter17mm(const Config& config, const Motors& motor) : Shooter(config), motor(motor) {}

void Shooter17mm::set_enable(const bool is_enable) {
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
    motor.shoot.set_enable(is_enable);
}

void Shooter17mm::update_ui(Ui& ui) {
    if (motor.rub1.is_connect == false) {
        ui.robot.rub1 = Ui::PINK;
        ui.robot.rub3 = Ui::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub1_measure_) > 1) {
        ui.robot.rub1 = Ui::ORANGE;
        ui.robot.rub3 = Ui::ORANGE;
    } else {
        ui.robot.rub1 = Ui::GREEN;
        ui.robot.rub3 = Ui::GREEN;
    }

    if (motor.rub2.is_connect == false) {
        ui.robot.rub2 = Ui::PINK;
        ui.robot.rub4 = Ui::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub2_measure_) > 1) {
        ui.robot.rub2 = Ui::ORANGE;
        ui.robot.rub4 = Ui::ORANGE;
    } else {
        ui.robot.rub2 = Ui::GREEN;
        ui.robot.rub4 = Ui::GREEN;
    }

    ui.robot.shoot = motor.shoot.is_connect ? Ui::GREEN : Ui::PINK;
}

void Shooter17mm::on_loop() {
    Shooter::on_loop();

    // 更新电机
    motor.rub1.on_loop();
    motor.rub2.on_loop();
    motor.shoot.on_loop();
}

void Shooter17mm::backward() {
    // 摩擦轮
    if (is_rub) {
        const float speed = bullet_speed.ref / config.rub_radius;
        motor.rub1.set_speed(speed);
        motor.rub2.set_speed(speed);
    } else {
        motor.rub1.set_speed(0.0f);
        motor.rub2.set_speed(0.0f);
    }

    // 拨弹电机
    if (is_rub && !is_heat_protect && is_shoot) { // 确保摩擦轮开启、枪口热量保护、发射信号
        if (!is_block_) {                         // 未卡弹
            // 正转
            const float shoot_speed = bullet_freq.ref / config.bullet_per_angle;
            motor.shoot.set_speed(shoot_speed);

            // 卡弹检测
            if (motor.shoot.current.ref >= motor.shoot.config.speed_pid_config->max_out) { // 达到速度环最大输出
                // 卡弹计时
                if (timer_block_.get_dt() > config.block_time) {
                    is_block_ = true;
                    timer_block_.update_dt(); // 重置计时器，用于倒转计时
                }
            } else {
                timer_block_.update_dt(); // 未卡弹，重置计时器
            }
        } else { // 卡弹
            // 反转
            const float shoot_speed = bullet_freq.ref / config.bullet_per_angle;
            motor.shoot.set_speed(-shoot_speed);

            // 反转计时
            if (timer_block_.get_dt() > config.reverse_time) {
                is_block_ = false;
                timer_block_.update_dt(); // 重置计时器，用于卡弹计时
            }
        }
    } else {
        is_block_ = false;           // 清除卡弹状态
        timer_block_.update_dt();    // 重置计时器
        motor.shoot.set_speed(0.0f); // 速度置零
    }
}

void Shooter17mm::forward() {
    // 摩擦轮
    rub1_measure_ = motor.rub1.speed.measure * config.rub_radius;
    rub2_measure_ = motor.rub2.speed.measure * config.rub_radius;
    bullet_speed.measure = (rub1_measure_ + rub2_measure_) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed.measure * config.bullet_per_angle;

    // 拨弹电机电流
    shoot_current = motor.shoot.current.measure;
}