#include "shooter_17mm.hpp"
#include <cmath>

Shooter17mm::Shooter17mm(const Config& config, const Motors& motor)
    : Shooter(config), motor(motor) {}

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

void Shooter17mm::update_ui(RobotUi& ui) {
    if (motor.rub1.is_connect == false) {
        ui.robot.rub1 = RobotUi::PINK;
        ui.robot.rub3 = RobotUi::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub1_measure_) > 1) {
        ui.robot.rub1 = RobotUi::ORANGE;
        ui.robot.rub3 = RobotUi::ORANGE;
    } else {
        ui.robot.rub1 = RobotUi::GREEN;
        ui.robot.rub3 = RobotUi::GREEN;
    }

    if (motor.rub2.is_connect == false) {
        ui.robot.rub2 = RobotUi::PINK;
        ui.robot.rub4 = RobotUi::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub2_measure_) > 1) {
        ui.robot.rub2 = RobotUi::ORANGE;
        ui.robot.rub4 = RobotUi::ORANGE;
    } else {
        ui.robot.rub2 = RobotUi::GREEN;
        ui.robot.rub4 = RobotUi::GREEN;
    }

    ui.robot.shoot = motor.shoot.is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

void Shooter17mm::update() {
    Shooter::update();

    // 更新电机
    motor.rub1.update();
    motor.rub2.update();
    motor.shoot.update();
}

void Shooter17mm::backward() {
    // 摩擦轮
    if (is_rub) {
        motor.rub1.set_enable(true);
        motor.rub2.set_enable(true);
        const float speed = bullet_speed.ref / config.rub_radius;
        const float rub1_output = rub_pid_.calculate(speed - rub1_measure_);
        const float rub2_output = rub_pid_.calculate(speed - rub2_measure_);
        if (config.rub_output == CURRENT_OUTPUT) {
            motor.rub1.set_current(rub1_output);
            motor.rub2.set_current(rub2_output);
        } else {
            motor.rub1.set_torque(rub1_output);
            motor.rub2.set_torque(rub2_output);
        }
    } else {
        rub_pid_.clear();
        motor.rub1.set_current(0);
        motor.rub2.set_current(0);
        if (config.auto_release_speed > 0) {
            motor.rub1.set_enable(std::fabs(motor.rub1.speed) >= config.auto_release_speed);
            motor.rub2.set_enable(std::fabs(motor.rub2.speed) >= config.auto_release_speed);
        }
    }

    // 拨弹电机
    if (is_rub && !is_heat_protect && is_shoot) { // 确保摩擦轮开启、枪口热量保护、发射信号
        if (!is_block_) {                         // 未卡弹
            // 正转
            const float shoot_speed = bullet_freq.ref / config.bullet_per_angle;
            const float shoot_output = shoot_pid_.calculate(shoot_speed - bullet_freq.measure);
            if (config.shoot_output == CURRENT_OUTPUT)
                motor.shoot.set_current(shoot_output);
            else
                motor.shoot.set_torque(shoot_output);

            // 卡弹检测
            if (motor.shoot.current_ref >= motor.shoot.config.drive.max_current) { // 达到电流限幅
                // 卡弹计时
                if (timer_block_.elapsed() > config.block_time) {
                    timer_block_.restart();
                    is_block_ = true;
                }
            } else {
                timer_block_.restart(); // 未卡弹，重置计时器
            }
        } else { // 卡弹
            // 反转
            const float shoot_speed = bullet_freq.ref / config.bullet_per_angle;
            const float shoot_output = shoot_pid_.calculate(-shoot_speed - bullet_freq.measure);
            if (config.shoot_output == CURRENT_OUTPUT)
                motor.shoot.set_current(shoot_output);
            else
                motor.shoot.set_torque(shoot_output);

            // 反转计时
            if (timer_block_.elapsed() > config.reverse_time) {
                timer_block_.restart();
                is_block_ = false;
            }
        }
    } else {
        is_block_ = false;      // 清除卡弹状态
        timer_block_.restart(); // 重置计时器
        shoot_pid_.clear();
        motor.shoot.set_current(0); // 电流置零
    }
}

void Shooter17mm::forward() {
    // 摩擦轮
    rub1_measure_ = motor.rub1.speed * config.rub_radius;
    rub2_measure_ = motor.rub2.speed * config.rub_radius;
    bullet_speed.measure = (rub1_measure_ + rub2_measure_) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed * config.bullet_per_angle;

    // 拨弹电机电流
    shoot_current = motor.shoot.current;
}
