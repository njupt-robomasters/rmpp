#include "motor.hpp"
#include <algorithm>
#include <cmath>

Motor::Motor(const Config& config)
    : config(config), speed_pid_(config.control.speed_pid_config), position_pid_(config.control.position_pid_config) {
    if (this->config.install.reduction == 0)
        this->config.install.reduction = 1;
    if (this->config.physical.kt == 0)
        this->config.physical.kt = 1;
}

Motor::Config Motor::apply_defaults(Config config, const Config& defaults) {
    if (config.install.reduction == 0)
        config.install.reduction = defaults.install.reduction;
    if (config.physical.kt == 0)
        config.physical.kt = defaults.physical.kt;
    if (config.physical.max_current == 0)
        config.physical.max_current = defaults.physical.max_current;
    if (config.physical.max_torque == 0)
        config.physical.max_torque = defaults.physical.max_torque;
    return config;
}

void Motor::subscribe() {
    config.protocol.can.add_rx_callback([this](const CanFrame& frame) { receive(frame); });
}

bool Motor::is_feedback_frame(const CanFrame& frame) const {
    return !frame.is_extend && !frame.is_remote && frame.size == 8 && frame.id == config.protocol.master_id;
}

void Motor::set_enable(bool enable) {
    if (is_enabled != enable) {
        is_enabled = enable;
        clear_control();
    }
}

void Motor::set_current(float value) {
    current.ref = value;
    torque.ref = value * config.physical.kt;
}

void Motor::set_torque(float value) {
    torque.ref = value;
    current.ref = value / config.physical.kt;
}

void Motor::set_speed(float value) {
    speed.ref = value;
}

bool Motor::is_multi_turn() const {
    return config.control.control_mode == MULTI_TURN_MODE || config.control.control_mode == MULTI_TURN_SPEED_MODE;
}

float Motor::selected_position() const {
    return is_multi_turn() ? position.measure_multi : position.measure;
}

float Motor::set_position(float value, float speed_ff) {
    update_connection();
    if (!is_connected || !is_enabled || !has_position_sample_ || position_pending_) {
        position.ref = selected_position();
        speed.ref = 0;
        position_pending_ = !is_connected || !is_enabled || !has_position_sample_;
        return position.ref;
    }
    const bool single_turn = !is_multi_turn();
    const float target = single_turn && !config.control.is_limit ? float(Angle(value)) : value;
    position.ref = config.control.is_limit ? std::clamp(target, config.control.position_min, config.control.position_max) : target;
    speed.ref = position.ref == value || (single_turn && !config.control.is_limit) ? speed_ff : 0;
    return position.ref;
}

bool Motor::set_offset(float offset) {
    if (!std::isfinite(offset))
        return false;
    config.install.offset = offset;
    if (has_position_sample_) {
        refresh_position();
        clear_control();
    }
    return true;
}

bool Motor::set_position_zero() {
    if (!has_position_sample_ || !is_connected)
        return false;
    return set_offset(accumulated_position_);
}

void Motor::clear_control() {
    speed_pid_.clear();
    position_pid_.clear();
    speed.outer = 0;
    position_pending_ = true;
    position.ref = selected_position();
    if (config.control.control_mode != OPEN_LOOP_MODE)
        set_current(0);
    if (config.control.control_mode != OPEN_LOOP_MODE && config.control.control_mode != SPEED_MODE)
        speed.ref = 0;
}

void Motor::update_connection() {
    if (is_connected && timer_connect_.elapsed() > config.protocol.timeout) {
        timer_connect_.restart();
        is_connected = false;
        clear_control();
    }
}

void Motor::mark_received() {
    update_connection();
    is_connected = true;
    timer_connect_.restart();
}

void Motor::update_control() {
    update_connection();
    if (!is_connected || !is_enabled) {
        clear_control();
        return;
    }
    if (config.control.control_mode == OPEN_LOOP_MODE)
        return;

    if (config.control.control_mode != SPEED_MODE && (position_pending_ || !has_position_sample_)) {
        set_current(0);
        return;
    }

    float result;
    if (config.control.control_mode == SPEED_MODE) {
        result = speed_pid_.calculate(speed.ref - speed.measure);
    } else {
        float error = position.ref - selected_position();
        if (!is_multi_turn() && !config.control.is_limit)
            error = Angle(error);
        const bool cascade = config.control.control_mode == SINGLE_TURN_SPEED_MODE || config.control.control_mode == MULTI_TURN_SPEED_MODE;
        if (cascade) {
            speed.outer = position_pid_.calculate(error);
            result = speed_pid_.calculate(speed.outer + speed.ref - speed.measure);
        } else {
            result = position_pid_.calculate(error, speed.ref - speed.measure);
        }
    }
    if (config.control.pid_output_type == CURRENT_OUTPUT)
        set_current(result);
    else
        set_torque(result);
}

void Motor::accept_feedback(const Raw& feedback) {
    mark_received();
    raw = feedback;
    const float direction = config.install.is_invert ? -1.0f : 1.0f;
    current.measure = direction * raw.current;
    torque.measure = direction * raw.torque;
    speed.measure = direction * raw.speed;
    record_position(raw.position);
}

void Motor::refresh_position() {
    position.measure_multi = (config.install.is_invert ? -1.0f : 1.0f) *
                             (accumulated_position_ - config.install.offset) / config.install.reduction;
    if (config.install.reduction == 1)
        position.measure = Angle((config.install.is_invert ? -1.0f : 1.0f) *
                                 (last_position_sample_ - config.install.offset));
    else
        position.measure = Angle(position.measure_multi);
}

void Motor::record_position(float sample) {
    raw.position = sample;
    if (has_position_sample_)
        accumulated_position_ += Angle(sample - last_position_sample_);
    else
        accumulated_position_ = sample;
    has_position_sample_ = true;
    last_position_sample_ = sample;
    refresh_position();
}

float Motor::output(float value, float maximum) const {
    if (!is_connected || !is_enabled || !std::isfinite(value))
        return 0;
    if (config.control.control_mode != OPEN_LOOP_MODE && config.control.control_mode != SPEED_MODE &&
        (position_pending_ || !has_position_sample_))
        return 0;
    return (config.install.is_invert ? -1.0f : 1.0f) * std::clamp(value, -maximum, maximum);
}

float Motor::output_current(float maximum) const {
    return output(current.ref, maximum);
}

float Motor::output_torque(float maximum) const {
    return output(torque.ref, maximum);
}
