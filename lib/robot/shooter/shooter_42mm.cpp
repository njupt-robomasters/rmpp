#include "shooter_42mm.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include <cmath>

Shooter42mm::Shooter42mm(const Config& config, const Motors& motor)
    : Shooter(config), motor(motor) {}

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

void Shooter42mm::update_ui(RobotUi& ui) {
    if (motor.rub1.is_connect == false) {
        ui.robot.rub1 = RobotUi::PINK;
    } else if (std::fabsf(bullet_speed.ref * RUB12_RATIO - rub1_measure_) < 1) {
        ui.robot.rub1 = RobotUi::GREEN;
    } else {
        ui.robot.rub1 = RobotUi::ORANGE;
    }

    if (motor.rub2.is_connect == false) {
        ui.robot.rub2 = RobotUi::PINK;
    } else if (std::fabsf(bullet_speed.ref * RUB12_RATIO - rub2_measure_) < 1) {
        ui.robot.rub2 = RobotUi::GREEN;
    } else {
        ui.robot.rub2 = RobotUi::ORANGE;
    }

    if (motor.rub3.is_connect == false) {
        ui.robot.rub3 = RobotUi::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub3_measure_) < 1) {
        ui.robot.rub3 = RobotUi::GREEN;
    } else {
        ui.robot.rub3 = RobotUi::ORANGE;
    }

    if (motor.rub4.is_connect == false) {
        ui.robot.rub4 = RobotUi::PINK;
    } else if (std::fabsf(bullet_speed.ref - rub4_measure_) < 1) {
        ui.robot.rub4 = RobotUi::GREEN;
    } else {
        ui.robot.rub4 = RobotUi::ORANGE;
    }

    ui.robot.shoot = motor.shoot.is_connect ? RobotUi::GREEN : RobotUi::PINK;
}

void Shooter42mm::update() {
    Shooter::update();

    // 更新电机
    motor.rub1.update();
    motor.rub2.update();
    motor.rub3.update();
    motor.rub4.update();
    motor.shoot.update();
}

void Shooter42mm::backward() {
    // 摩擦轮
    if (is_rub) {
        motor.rub1.set_enable(true);
        motor.rub2.set_enable(true);
        motor.rub3.set_enable(true);
        motor.rub4.set_enable(true);
        const float speed = bullet_speed.ref / config.rub_radius;
        const float rub1_output = rub_pid_.calculate(speed * RUB12_RATIO - rub1_measure_);
        const float rub2_output = rub_pid_.calculate(speed * RUB12_RATIO - rub2_measure_);
        const float rub3_output = rub_pid_.calculate(speed - rub3_measure_);
        const float rub4_output = rub_pid_.calculate(speed - rub4_measure_);
        if (config.rub_output == CURRENT_OUTPUT) {
            motor.rub1.set_current(rub1_output);
            motor.rub2.set_current(rub2_output);
            motor.rub3.set_current(rub3_output);
            motor.rub4.set_current(rub4_output);
        } else {
            motor.rub1.set_torque(rub1_output);
            motor.rub2.set_torque(rub2_output);
            motor.rub3.set_torque(rub3_output);
            motor.rub4.set_torque(rub4_output);
        }
    } else {
        rub_pid_.clear();
        motor.rub1.set_current(0);
        motor.rub2.set_current(0);
        motor.rub3.set_current(0);
        motor.rub4.set_current(0);
        if (config.auto_release_speed > 0) {
            motor.rub1.set_enable(std::fabs(motor.rub1.speed) >= config.auto_release_speed);
            motor.rub2.set_enable(std::fabs(motor.rub2.speed) >= config.auto_release_speed);
            motor.rub3.set_enable(std::fabs(motor.rub3.speed) >= config.auto_release_speed);
            motor.rub4.set_enable(std::fabs(motor.rub4.speed) >= config.auto_release_speed);
        }
    }

    // 拨弹电机
    if (!is_rub) { // 关摩擦轮
        state_ = FREE;
    }

    switch (state_) {
    case FREE: {
        if (config.shoot_output == CURRENT_OUTPUT)
            motor.shoot.set_current(0);
        else
            motor.shoot.set_torque(0); // 保持当前状态

        if (is_rub) { // 开摩擦轮
            state_ = SHAKE;
            timer_shake_.restart();
            shake_upper_ = motor.shoot.position;
        }
    } break;

    case SHAKE: {
        const Angle angle_per_bullet = 1 / config.bullet_per_angle;
        const float a = angle_per_bullet * SHANE_AMP / 2;
        const float offset = shake_upper_ - a;
        const float t = timer_shake_.elapsed();
        const Angle angle = a * std::cosf(2 * pi * SHAKE_FREQ * t) + offset;
        const float shoot_output = shoot_pid_.calculate(angle - motor.shoot.position);
        if (config.shoot_output == CURRENT_OUTPUT)
            motor.shoot.set_current(shoot_output);
        else
            motor.shoot.set_torque(shoot_output);

        // 防止计时器溢出
        if (SHAKE_FREQ * t > 1) {
            timer_shake_.restart();
        }

        if (is_shoot && !is_shoot_last_ && !is_heat_protect) { // 发射信号上升沿，热量保护
            // 增加角度
            const Angle angle_per_bullet = 1 / config.bullet_per_angle;
            Angle target_angle = shake_upper_ + angle_per_bullet;

            // 对齐相位
            const int8_t index = (int8_t)std::round((target_angle / angle_per_bullet)); // 四舍五入
            target_angle = index * angle_per_bullet;

            // 设置电机角度
            const float shoot_output = shoot_pid_.calculate(target_angle - motor.shoot.position);
            if (config.shoot_output == CURRENT_OUTPUT)
                motor.shoot.set_current(shoot_output);
            else
                motor.shoot.set_torque(shoot_output);

            state_ = SHOOT;
            timer_block_.restart();
        }
    } break;

    case SHOOT: {
        // 拨弹计时
        const Angle angle_per_bullet = 1 / config.bullet_per_angle;
        const Angle target_angle = shake_upper_ + angle_per_bullet;
        const float shoot_output = shoot_pid_.calculate(target_angle - motor.shoot.position);
        if (config.shoot_output == CURRENT_OUTPUT)
            motor.shoot.set_current(shoot_output);
        else
            motor.shoot.set_torque(shoot_output);

        if (std::fabsf(target_angle - motor.shoot.position) < (1.0f - SHOOT_FINISH_RATIO) * angle_per_bullet) {
            state_ = SHAKE;
            timer_shake_.restart();
            shake_upper_ = target_angle;
        }

        // 卡弹检测
        if (motor.shoot.torque_ref >= motor.shoot.config.drive.max_torque) { // 达到力矩限幅
            if (timer_block_.elapsed() > config.block_time) {                // 卡弹计时
                timer_block_.restart();
                state_ = SHAKE;
                timer_shake_.restart();
                shake_upper_ = motor.shoot.position;
            }
        } else {
            timer_block_.restart(); // 未卡弹，重置计时器
        }
    } break;

    default:
        break;
    }
    is_shoot_last_ = is_shoot;
}

void Shooter42mm::forward() {
    // 摩擦轮
    rub1_measure_ = motor.rub1.speed * config.rub_radius;
    rub2_measure_ = motor.rub2.speed * config.rub_radius;
    rub3_measure_ = motor.rub3.speed * config.rub_radius;
    rub4_measure_ = motor.rub4.speed * config.rub_radius;
    bullet_speed.measure = (rub3_measure_ + rub4_measure_) / 2.0f;

    // 拨弹电机
    bullet_freq.measure = motor.shoot.speed * config.bullet_per_angle;

    // 拨弹电机电流
    shoot_current = motor.shoot.current;
}
