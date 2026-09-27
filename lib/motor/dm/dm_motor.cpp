#include "dm_motor.hpp"

DmMotor::DmMotor(const Config& config, Protocol protocol)
    : MitMotor(config, protocol) {}

void DmMotor::receive(const CanFrame& frame) {
    if (!is_feedback_frame(frame) || (frame.data[0] & 0x0f) != (config.protocol.slave_id & 0x0f))
        return;
    const auto* data = frame.data;
    err_code = static_cast<ErrCode>(data[0] >> 4);
    temperature.mos = static_cast<int8_t>(data[6]);
    temperature.motor = static_cast<int8_t>(data[7]);
    accept_mit_feedback(data, TORQUE_OUTPUT);
}

void DmMotor::set_enable(bool enable) {
    if (is_enabled != enable) {
        enable_submitted_ = false;
        send_count_ = 0;
    }
    Motor::set_enable(enable);
}

bool DmMotor::send_can_cmd() {
    update_connection();
    if (!is_enabled) {
        enable_submitted_ = false;
        send_count_ = 0;
        return send_state(0xfd);
    }
    // 启动和断联恢复期间只请求使能，有反馈后才发送力矩
    if (!enable_submitted_ || !is_connected || send_count_ == 99) {
        if (!send_state(0xfc))
            return false;
        enable_submitted_ = true;
        send_count_ = 0;
        return true;
    }
    if (!send_mit(TORQUE_OUTPUT))
        return false;
    ++send_count_;
    return true;
}
