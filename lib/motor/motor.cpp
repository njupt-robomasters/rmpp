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

Motor::Motor(const Config& config, const float max_current, const float max_torque, const float reduction)
    : config(config) {
    if (this->config.max_current == 0)
        this->config.max_current = max_current;
    if (this->config.max_torque == 0)
        this->config.max_torque = max_torque;
    if (this->config.reduction == 0)
        this->config.reduction = reduction;
}

void Motor::can_callback(const bool is_angle_frame) {
    // 用于断联检测
    is_connect = true;
    callback_timer.restart();

    // 首帧保留编码器零点，后续累计跨圈增量
    if (is_angle_frame) {
        if (is_angle_initialized)
            raw_angle_multi += Angle(raw_angle - raw_angle_last);
        else {
            raw_angle_multi = raw_angle;
            is_angle_initialized = true;
        }
        raw_angle_last = raw_angle;
    }

    // 安装方向修正
    const float dir = config.is_invert ? -1.0f : 1.0f;
    current = dir * raw_current;
    torque = dir * raw_torque;
    speed = dir * raw_speed / config.reduction;

    // 单圈与多圈反馈共用电机侧零点
    angle_multi = dir * (raw_angle_multi - config.offset) / config.reduction;
    if (config.reduction == 1)
        angle = Angle(dir * (raw_angle - config.offset));
    else
        angle = Angle(angle_multi);
}

void Motor::update() {
    // 断联检测：清零电流、力矩、速度反馈，保持角度
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        current = torque = speed = 0;
    }
}
