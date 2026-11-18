#include "motor.hpp"
#include "math/angle.hpp"
#include <algorithm>

Motor::Motor(const Config& config, const float max_current, const float max_torque, const float reduction)
    : config(config) {
    if (this->config.max_current == 0)
        this->config.max_current = max_current;
    if (this->config.max_torque == 0)
        this->config.max_torque = max_torque;
    if (this->config.reduction == 0)
        this->config.reduction = reduction;
}

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
    // 断联检测：清零电流、力矩、速度反馈，保持角度
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        current = torque = speed = 0;
    }
}

void Motor::can_callback(const Feedback& feedback) {
    // 用于断联检测
    is_connect = true;
    callback_timer.restart();

    // 安装方向与减速比修正；电流与力矩只传其一时按量程换算另一项，未携带的量保持上次反馈
    const float dir = config.is_invert ? -1.0f : 1.0f;
    if (feedback.current)
        current = dir * *feedback.current;
    if (feedback.torque)
        torque = dir * *feedback.torque;
    if (feedback.current && !feedback.torque)
        torque = current / config.max_current * config.max_torque;
    if (feedback.torque && !feedback.current)
        current = torque / config.max_torque * config.max_current;
    if (feedback.speed)
        speed = dir * *feedback.speed / config.reduction;

    // 首个角度保留编码器零点，后续累计跨圈增量
    if (feedback.angle) {
        if (is_angle_initialized) {
            raw_angle_multi += Angle(*feedback.angle - raw_angle);
        } else {
            raw_angle_multi = *feedback.angle;
            is_angle_initialized = true;
        }
        raw_angle = *feedback.angle;
    }

    // 单圈与多圈反馈共用电机侧零点，每次回调按当前 offset 刷新
    angle_multi = dir * (raw_angle_multi - config.offset) / config.reduction;
    if (config.reduction == 1)
        angle = Angle(dir * (raw_angle - config.offset));
    else
        angle = Angle(angle_multi);
}
