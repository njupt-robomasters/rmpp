#include "shooter_17mm.hpp"
#include <algorithm>
#include <cmath>

Shooter17mm::Shooter17mm(const Motors& motor, const Config& config)
    : motor(motor), config(config),
      shoot_pid(config.shoot_pid_config) {}

void Shooter17mm::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
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
    is_heat_protect = heat_remain < config.heat_protect;
}

void Shooter17mm::update() {
    const float dt = std::clamp(update_timer.restart(), DT_MIN, DT_MAX);

    // 离线检测
    motor.rub1.update();
    motor.rub2.update();
    motor.shoot.update();

    // 弹速与弹频反馈
    rub_speed[0] = motor.rub1.speed * config.rub_radius;
    rub_speed[1] = motor.rub2.speed * config.rub_radius;
    bullet_speed.measure = (rub_speed[0] + rub_speed[1]) / 2.0f;
    bullet_freq.measure = motor.shoot.speed * config.bullet_per_angle;

    // 摩擦轮速度控制
    const float rub_speed_ref = is_enable ? bullet_speed.ref / config.rub_radius : 0.0f;
    motor.rub1.set_speed(rub_speed_ref);
    motor.rub2.set_speed(rub_speed_ref);

    // 拨弹启停判断
    if (!is_enable || is_heat_protect || bullet_freq.ref == 0 || !motor.shoot.is_connect) {
        state = STOP;
    } else if (state == STOP) {
        state = FEED;
    }

    // 拨弹与卡弹退弹状态机
    float shoot_speed = bullet_freq.ref / config.bullet_per_angle;
    switch (state) {
    case STOP:
        shoot_speed = 0;
        block_elapsed = reverse_elapsed = 0;
        break;

    case FEED:
        // 上一周期 PID 输出未饱和时清零计时，饱和持续 block_time 后反转
        if (std::fabs(shoot_output) < config.shoot_pid_config->max_out) {
            block_elapsed = 0;
        } else {
            block_elapsed += dt;
            if (block_elapsed >= config.block_time) {
                state = REVERSE;
                reverse_elapsed = 0;
            }
        }
        break;

    case REVERSE:
        shoot_speed = -shoot_speed;
        reverse_elapsed += dt;
        if (reverse_elapsed >= config.reverse_time) {
            state = FEED;
            block_elapsed = 0;
        }
        break;
    }

    // 拨弹速度闭环与失能处理
    if (is_enable) {
        shoot_output = shoot_pid.calculate(dt, shoot_speed - motor.shoot.speed);
    } else {
        shoot_output = 0;
        shoot_pid.clear();
    }
    motor.shoot.set_output(shoot_output);
}
