#include "motor.hpp"
#include "math/angle.hpp"
#include <algorithm>

void Motor::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
}

void Motor::set_current(float current) {
    if (!std::isfinite(current))
        current = 0;
    current = std::clamp(current, -max_current, max_current);
    current_ref = current;
    torque_ref = current / max_current * max_torque;
}

void Motor::set_torque(float torque) {
    if (!std::isfinite(torque))
        torque = 0;
    torque = std::clamp(torque, -max_torque, max_torque);
    torque_ref = torque;
    current_ref = torque / max_torque * max_current;
}

void Motor::set_offset(const float offset) {
    this->offset = offset;
}

void Motor::set_zero() {
    const float dir = is_invert ? -1.0f : 1.0f;
    set_offset(offset + dir * position_multi * reduction);
}

void Motor::update() {
    // 断联检测：清零电流、力矩、速度反馈，保持位置
    if (callback_timer.elapsed() > timeout) {
        is_connect = false;
        current = torque = speed = 0;
    }
}

Motor::Motor(const Config& config, const float max_current, const float max_torque, const float reduction)
    : offset(config.offset),
      max_current(config.max_current == 0 ? max_current : config.max_current),
      max_torque(config.max_torque == 0 ? max_torque : config.max_torque),
      can(config.can),
      tx_id(config.tx_id),
      rx_id(config.rx_id),
      is_invert(config.is_invert),
      reduction(config.reduction == 0 ? reduction : config.reduction),
      timeout(config.timeout) {}

void Motor::can_callback() {
    // 用于断联检测
    is_connect = true;
    callback_timer.restart();

    // 用于多圈计数
    raw_position_multi += Angle(raw_position - raw_position_last);
    raw_position_last = raw_position;

    // 安装方向修正
    const float dir = is_invert ? -1.0f : 1.0f;
    current = dir * raw_current;
    torque = dir * raw_torque;
    speed = dir * raw_speed / reduction;
    position_multi = dir * (raw_position_multi - offset) / reduction;

    // 减速比为1，使用单圈计数
    // 减速比不为1，使用多圈计数归一化
    if (reduction == 1)
        position = Angle(dir * (raw_position - offset));
    else
        position = Angle(position_multi);
}
