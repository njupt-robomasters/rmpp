#include "shooter_42mm.hpp"
#include <cmath>

Shooter42mm::Shooter42mm(const Motors& motor, const Config& config)
    : motor(motor), config_(config), shoot_pid_(config.shoot_pid_config) {}

void Shooter42mm::set_enable(const bool is_enable) {
    is_enable_ = is_enable;
    motor.rub1.set_enable(is_enable);
    motor.rub2.set_enable(is_enable);
    motor.rub3.set_enable(is_enable);
    motor.rub4.set_enable(is_enable);
    motor.shoot.set_enable(is_enable);
}

void Shooter42mm::set_bullet_speed(const float bullet_speed) {
    this->bullet_speed.ref = bullet_speed;
}

void Shooter42mm::set_shoot(const bool is_shoot) {
    is_shoot_ = is_shoot;
}

void Shooter42mm::set_heat_remain(const int32_t heat_remain) {
    is_heat_protect_ = heat_remain < config_.heat_protect;
}

void Shooter42mm::update() {
    // 离线检测
    motor.rub1.update();
    motor.rub2.update();
    motor.rub3.update();
    motor.rub4.update();
    motor.shoot.update();

    // 正解：电机速度 -> 弹速
    rub_speed[0] = motor.rub1.speed * config_.rub_radius;
    rub_speed[1] = motor.rub2.speed * config_.rub_radius;
    rub_speed[2] = motor.rub3.speed * config_.rub_radius;
    rub_speed[3] = motor.rub4.speed * config_.rub_radius;
    bullet_speed.measure = (rub_speed[2] + rub_speed[3]) / 2.0f;

    // 摩擦轮：VESC 速度模式，关闭时目标为零制动；低速释放由 app 处理
    const bool is_rub = is_enable_ && bullet_speed.ref > 0;
    const float rub_speed_ref = is_rub ? bullet_speed.ref / config_.rub_radius : 0.0f;
    motor.rub1.set_speed(rub_speed_ref * RUB12_RATIO);
    motor.rub2.set_speed(rub_speed_ref * RUB12_RATIO);
    motor.rub3.set_speed(rub_speed_ref);
    motor.rub4.set_speed(rub_speed_ref);

    // 拨弹电机：关摩擦轮或离线时回到空闲，重连后从当前角度重新开始抖动
    if (!is_rub || !motor.shoot.is_connect) {
        state_ = FREE;
        shoot_pid_.clear();
    }

    const Angle angle_per_bullet = 1 / config_.bullet_per_angle;
    switch (state_) {
    case FREE: {
        motor.shoot.set_current(0);

        if (is_rub && motor.shoot.is_connect) { // 开摩擦轮
            state_ = SHAKE;
            timer_shake_.restart();
            shake_upper_ = motor.shoot.position;
        }
    } break;

    case SHAKE: {
        const float a = angle_per_bullet * SHAKE_AMP / 2;
        const float offset = shake_upper_ - a;
        const float t = timer_shake_.elapsed();
        const Angle angle = a * std::cos(2 * pi * SHAKE_FREQ * t) + offset;
        set_shoot_output(shoot_pid_.calculate(angle - motor.shoot.position));

        // 防止计时器溢出
        if (SHAKE_FREQ * t > 1)
            timer_shake_.restart();

        // 发射信号上升沿且未热量保护时拨一发，目标对齐到整发相位
        if (is_shoot_ && !last_shoot_ && !is_heat_protect_) {
            const float index = std::round((shake_upper_ + angle_per_bullet) / angle_per_bullet);
            shake_upper_ = index * angle_per_bullet - angle_per_bullet;
            state_ = SHOOT;
            timer_block_.restart();
        }
    } break;

    case SHOOT: {
        const Angle target_angle = shake_upper_ + angle_per_bullet;
        set_shoot_output(shoot_pid_.calculate(target_angle - motor.shoot.position));

        // 到位后回到抖动
        if (std::fabs(target_angle - motor.shoot.position) < (1.0f - SHOOT_FINISH_RATIO) * angle_per_bullet) {
            state_ = SHAKE;
            timer_shake_.restart();
            shake_upper_ = target_angle;
        }

        // 达到力矩限幅并持续 block_time 判为卡弹，从当前角度重新抖动
        if (std::fabs(motor.shoot.torque_ref) < motor.shoot.config.drive.max_torque) {
            timer_block_.restart();
        } else if (timer_block_.elapsed() > config_.block_time) {
            timer_block_.restart();
            state_ = SHAKE;
            timer_shake_.restart();
            shake_upper_ = motor.shoot.position;
        }
    } break;
    }
    last_shoot_ = is_shoot_;
}

void Shooter42mm::set_shoot_output(const float output) {
    if (config_.shoot_output == CURRENT_OUTPUT) {
        motor.shoot.set_current(output);
        shoot_pid_.track(motor.shoot.current_ref);
    } else {
        motor.shoot.set_torque(output);
        shoot_pid_.track(motor.shoot.torque_ref);
    }
}
