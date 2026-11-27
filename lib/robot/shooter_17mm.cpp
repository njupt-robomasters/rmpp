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

void Shooter17mm::set_heat(const int32_t heat_remain, const float cooling_rate) {
    heat.remain = heat_remain;
    heat.cooling = cooling_rate;
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

    // 热量限制：延迟内按目标弹频净耗的热量作为降频目标，低于目标时按热量的偏差线性降频，弹频不低于 0
    const float heat_ref = std::max(bullet_freq.ref, 0.0f);

    heat.target = (HEAT_PER_SHOT * heat_ref - heat.cooling) * config.heat_delay + config.heat_reserve;
    const float heat_freq = heat.cooling / HEAT_PER_SHOT + config.heat_gain * (heat.remain - heat.target);
    bullet_freq.effective = std::min(bullet_freq.ref, std::max(heat_freq, 0.0f));
    is_heat_limit = bullet_freq.effective < bullet_freq.ref;

    // 摩擦轮速度控制
    const float rub_speed_ref = is_enable ? bullet_speed.ref / config.rub_radius : 0.0f;
    motor.rub1.set_speed(rub_speed_ref);
    motor.rub2.set_speed(rub_speed_ref);

    // 拨弹启停判断
    if (!is_enable || bullet_freq.effective == 0 || !motor.shoot.is_connect) {
        state = STOP;
    } else if (state == STOP) {
        state = FEED;
    }

    // 拨弹与卡弹退弹状态机
    float shoot_speed = bullet_freq.effective / config.bullet_per_angle;
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
