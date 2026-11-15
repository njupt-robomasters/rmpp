#include "motor.hpp"
#include "math/angle.hpp"
#include <algorithm>

void Motor::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
}

void Motor::set_output(const float output) {
    set_current(output);
}

void Motor::set_current(float current) {
    if (!std::isfinite(current))
        current = 0;
    current = std::clamp(current, -config.max_current, config.max_current);
    current_ref = current;
    torque_ref = current / config.max_current * config.max_torque;
}

void Motor::set_torque(float torque) {
    if (!std::isfinite(torque))
        torque = 0;
    torque = std::clamp(torque, -config.max_torque, config.max_torque);
    torque_ref = torque;
    current_ref = torque / config.max_torque * config.max_current;
}

void Motor::update() {
    // 断联检测：清零电流、力矩、速度反馈，保持位置
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        current = torque = speed = 0;
    }
}

Motor::Motor(const Config& config, const float max_current, const float max_torque, const float reduction)
    : config(config) {
    if (this->config.max_current == 0)
        this->config.max_current = max_current;
    if (this->config.max_torque == 0)
        this->config.max_torque = max_torque;
    if (this->config.reduction == 0)
        this->config.reduction = reduction;
}

void Motor::can_callback() {
    // 用于断联检测
    is_connect = true;
    callback_timer.restart();

    // 用于多圈计数
    raw_position_multi += Angle(raw_position - raw_position_last);
    raw_position_last = raw_position;

    // 安装方向修正
    const float dir = config.is_invert ? -1.0f : 1.0f;
    current = dir * raw_current;
    torque = dir * raw_torque;
    speed = dir * raw_speed / config.reduction;

    // 减速比为1，使用单圈计数
    // 减速比不为1，使用多圈计数归一化
    if (config.reduction == 1)
        position = Angle(dir * (raw_position - config.offset));
    else
        position = Angle(dir * (raw_position_multi - config.offset) / config.reduction);
}
