#include "shooter_17mm.hpp"
#include <cmath>

Shooter17mm::Shooter17mm(const Config& config, const Motors& motor)
    : Shooter(config), motor(motor), rub1_pid_(config.rub_pid_config), rub2_pid_(config.rub_pid_config) {}

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

void Shooter17mm::set_motors_enable(const bool is_enable) {
    motor.rub1.set_enable(is_enable);
    motor.rub2.set_enable(is_enable);
    motor.shoot.set_enable(is_enable);
}

void Shooter17mm::update_motors() {
    motor.rub1.update();
    motor.rub2.update();
    motor.shoot.update();
}

void Shooter17mm::backward() {
    // 摩擦轮，各自处理离线
    const float speed = bullet_speed.ref / config.rub_radius;
    control_rub(motor.rub1, rub1_pid_, speed, motor.rub1.speed);
    control_rub(motor.rub2, rub2_pid_, speed, motor.rub2.speed);

    // 拨弹电机：摩擦轮开启、未热量保护、有发射信号且在线；不检查摩擦轮是否在线，便于单独调试
    if (!is_rub || is_heat_protect || !is_shoot || !motor.shoot.is_connect) {
        is_block_ = false;
        timer_block_.restart();
        shoot_pid_.clear();
        motor.shoot.set_current(0);
        return;
    }

    // 未卡弹正转，卡弹反转
    const float shoot_speed = bullet_freq.ref / config.bullet_per_angle;
    const float shoot_speed_ref = is_block_ ? -shoot_speed : shoot_speed;
    set_output(motor.shoot, config.shoot_output, shoot_pid_.calculate(shoot_speed_ref - motor.shoot.speed));

    if (!is_block_) {
        // 达到电流限幅并持续 block_time 判为卡弹
        if (motor.shoot.current_ref < motor.shoot.config.drive.max_current) {
            timer_block_.restart();
        } else if (timer_block_.elapsed() > config.block_time) {
            timer_block_.restart();
            is_block_ = true;
        }
    } else if (timer_block_.elapsed() > config.reverse_time) {
        timer_block_.restart();
        is_block_ = false;
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
