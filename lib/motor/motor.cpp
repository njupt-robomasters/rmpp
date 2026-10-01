#include "motor.hpp"
#include "math/angle.hpp"
#include <algorithm>
#include <cmath>

Motor::Config Motor::Config::with_defaults(const Drive& drive) const {
    Config config = *this;
    if (config.drive.kt == 0)
        config.drive.kt = drive.kt;
    if (config.drive.max_current == 0 && config.drive.max_torque == 0) {
        config.drive.max_current = drive.max_current;
        config.drive.max_torque = drive.max_torque;
    }
    if (config.drive.reduction == 0)
        config.drive.reduction = drive.reduction;
    return config;
}

void Motor::set_enable(const bool is_enable) {
    this->is_enable = is_enable;
}

void Motor::set_current(float current) {
    if (!std::isfinite(current))
        current = 0;
    current = std::clamp(current, -config_.drive.max_current, config_.drive.max_current);
    this->current.ref = current;
    torque.ref = current * config_.drive.kt;
}

void Motor::set_torque(float torque) {
    if (!std::isfinite(torque))
        torque = 0;
    torque = std::clamp(torque, -config_.drive.max_torque, config_.drive.max_torque);
    this->torque.ref = torque;
    current.ref = torque / config_.drive.kt;
}

void Motor::set_speed(const float speed) {
    this->speed.ref = speed;
}

float Motor::set_position(float position, const float speed_ff) {
    const bool is_single = config_.control.mode == SINGLE_TURN_POSITION_MODE || config_.control.mode == SINGLE_TURN_POSITION_SPEED_MODE;
    if (is_single && !config_.control.is_limit) {
        position = Angle(position);
    }

    if (config_.control.is_limit) {
        if (position < config_.control.position_min) {
            position = config_.control.position_min;
        } else if (position > config_.control.position_max) {
            position = config_.control.position_max;
        } else { // 未触发限位时使用前馈
            speed.ff = speed_ff;
        }
    } else { // 未触发限位时使用前馈
        speed.ff = speed_ff;
    }

    this->position.ref = position;

    // 返回实际设置的位置
    return position;
}

void Motor::set_offset(const float offset) {
    config_.install.offset = offset;
}

void Motor::update() {
    // 离线检测
    if (rx_timer_.elapsed() > config_.bus.timeout)
        is_connect = false;

    // 离线或未使能，清空PID
    if (!is_connect || !is_enable) {
        speed_pid_.clear();
        position_pid_.clear();
        return;
    }

    // 开环模式无需PID计算
    if (config_.control.mode == OPEN_LOOP_MODE)
        return;

    // PID计算
    float result;
    if (config_.control.mode == SPEED_MODE) {
        result = speed_pid_.calculate(speed.ref - speed.measure);
    } else {
        // 单圈模式/多圈模式
        const bool is_single = config_.control.mode == SINGLE_TURN_POSITION_MODE || config_.control.mode == SINGLE_TURN_POSITION_SPEED_MODE;
        // 串级PID/单级PID
        const bool is_cascade = config_.control.mode == SINGLE_TURN_POSITION_SPEED_MODE || config_.control.mode == MULTI_TURN_POSITION_SPEED_MODE;

        // 计算误差
        float error;
        if (is_single && !config_.control.is_limit) {
            error = Angle(position.ref - position.measure);
        } else {
            error = position.ref - position.measure_multi;
        }

        // 计算PID
        if (is_cascade) { // 串级PID
            speed.ref = position_pid_.calculate(error) + speed.ff;
            result = speed_pid_.calculate(speed.ref - speed.measure);
        } else { // 单级PID
            speed.ref = speed.ff;
            result = position_pid_.calculate(error, speed.ref - speed.measure);
        }
    }
    if (config_.control.pid_output_type == CURRENT_OUTPUT)
        set_current(result);
    else
        set_torque(result);
}

Motor::Motor(const Config& config)
    : config_(config),
      speed_pid_(config.control.speed_pid_config),
      position_pid_(config.control.position_pid_config) {

    // kt 默认值为 1
    if (config_.drive.kt == 0)
        config_.drive.kt = 1;

    // reduction 默认值为 1
    if (config_.drive.reduction == 0)
        config_.drive.reduction = 1;

    // max_current 和 max_torque 换算：不全为 0 时用 kt 换算
    if (config_.drive.max_current != 0 || config_.drive.max_torque != 0) {
        if (config_.drive.max_current == 0)
            config_.drive.max_current = config_.drive.max_torque / config_.drive.kt;
        else if (config_.drive.max_torque == 0)
            config_.drive.max_torque = config_.drive.max_current * config_.drive.kt;
    }
}

void Motor::can_callback() {
    // 用于离线检测
    is_connect = true;
    rx_timer_.restart();

    // 原始位置累计
    raw_position_multi_ += Angle(raw_position_last_ - position.measure);
    raw_position_last_ = raw.position;

    // 修正方向
    const float dir = config_.install.is_invert ? -1.0f : 1.0f;
    current.measure = dir * raw.current;
    torque.measure = dir * raw.torque;
    speed.measure = dir * raw.speed;

    // 多圈计数
    position.measure_multi = dir * (raw_position_multi_ - config_.install.offset) / config_.drive.reduction;

    // 单圈计数
    if (config_.drive.reduction == 1) // 无减速时，直接用单圈原始值
        position.measure = Angle(dir * (raw.position - config_.install.offset));
    else // 有减速时，使用多圈计数归一化
        position.measure = Angle(position.measure_multi);
}
