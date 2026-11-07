#include "motor.hpp"
#include "math/angle.hpp"
#include <algorithm>

void Motor::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
}

void Motor::set_current(float current) {
    if (!std::isfinite(current))
        current = 0;
    current = std::clamp(current, -config.drive.max_current, config.drive.max_current);
    current_ref = current;
    torque_ref = current * config.drive.kt;
}

void Motor::set_torque(float torque) {
    if (!std::isfinite(torque))
        torque = 0;
    torque = std::clamp(torque, -config.drive.max_torque, config.drive.max_torque);
    torque_ref = torque;
    current_ref = torque / config.drive.kt;
}

void Motor::set_offset(const float offset) {
    config.install.offset = offset;
}

void Motor::update() {
    // 断联检测：清零电流、力矩、速度反馈，保持位置
    if (rx_timer_.elapsed() > config.bus.timeout) {
        is_connect = false;
        current = torque = speed = 0;
    }
}

Motor::Motor(const Config& config)
    : config(config) {
}

void Motor::can_callback() {
    // 用于断联检测
    is_connect = true;
    rx_timer_.restart();

    // 用于多圈计数
    raw_position_multi_ += Angle(raw_position_last_ - raw.position);
    raw_position_last_ = raw.position;

    // 安装方向修正
    const float dir = config.install.is_invert ? -1.0f : 1.0f;
    current = dir * raw.current;
    torque = dir * raw.torque;
    speed = dir * raw.speed / config.drive.reduction;
    position_multi = dir * (raw_position_multi_ - config.install.offset) / config.drive.reduction;

    // 减速比为1，使用单圈计数
    // 减速比不为1，使用多圈计数归一化
    if (config.drive.reduction == 1)
        position = Angle(dir * (raw.position - config.install.offset));
    else
        position = Angle(position_multi);
}
