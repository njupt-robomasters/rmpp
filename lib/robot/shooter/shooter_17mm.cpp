#include "shooter_17mm.hpp"
#include <cmath>

Shooter17mm::Shooter17mm(const Motors& motor, const Config& config)
    : motor(motor), config_(config), shoot_pid_(config.shoot_pid_config) {}

void Shooter17mm::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    motor.rub1.set_enable(is_enable);
    motor.rub2.set_enable(is_enable);
    motor.shoot.set_enable(is_enable);
}

void Shooter17mm::set_bullet_speed(const float bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter17mm::set_bullet_freq(const float bullet_freq) {
    this->bullet_freq.ref = bullet_freq;
}

void Shooter17mm::set_heat_remain(const int32_t heat_remain) {
    is_heat_protect_ = heat_remain < config_.heat_protect;
}

void Shooter17mm::update() {
    // 离线检测
    motor.rub1.update();
    motor.rub2.update();
    motor.shoot.update();

    // 正解：电机速度 -> 弹速、弹频
    rub_speed[0] = motor.rub1.speed * config_.rub_radius;
    rub_speed[1] = motor.rub2.speed * config_.rub_radius;
    bullet_speed.measure = (rub_speed[0] + rub_speed[1]) / 2.0f;
    bullet_freq.measure = motor.shoot.speed * config_.bullet_per_angle;

    // 摩擦轮：VESC 速度模式，关闭时目标为零制动；低速释放由 app 处理
    const bool is_rub = is_enable_ && bullet_speed.ref > 0;
    const float rub_speed_ref = is_rub ? bullet_speed.ref / config_.rub_radius : 0.0f;
    motor.rub1.set_speed(rub_speed_ref);
    motor.rub2.set_speed(rub_speed_ref);

    // 拨弹电机：摩擦轮开启、未热量保护、弹频非零且在线；不检查摩擦轮是否在线，便于单独调试
    if (!is_rub || is_heat_protect_ || bullet_freq.ref == 0 || !motor.shoot.is_connect) {
        is_block_ = false;
        timer_block_.restart();
        shoot_pid_.clear();
        motor.shoot.set_current(0);
        return;
    }

    // 未卡弹按弹频转动，卡弹反转
    const float shoot_speed = bullet_freq.ref / config_.bullet_per_angle;
    const float shoot_speed_ref = is_block_ ? -shoot_speed : shoot_speed;
    const float shoot_output = shoot_pid_.calculate(shoot_speed_ref - motor.shoot.speed);
    if (config_.shoot_output == CURRENT_OUTPUT) {
        motor.shoot.set_current(shoot_output);
        shoot_pid_.track(motor.shoot.current_ref);
    } else {
        motor.shoot.set_torque(shoot_output);
        shoot_pid_.track(motor.shoot.torque_ref);
    }

    if (!is_block_) {
        // 达到电流限幅并持续 block_time 判为卡弹；取绝对值兼顾负弹频反转
        if (std::fabs(motor.shoot.current_ref) < motor.shoot.config.drive.max_current) {
            timer_block_.restart();
        } else if (timer_block_.elapsed() > config_.block_time) {
            timer_block_.restart();
            is_block_ = true;
        }
    } else if (timer_block_.elapsed() > config_.reverse_time) {
        timer_block_.restart();
        is_block_ = false;
    }
}
