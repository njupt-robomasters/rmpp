#include "dm_motor.hpp"
#include <algorithm>

static float decode(const uint16_t value, const float maximum, const unsigned bits) {
    return static_cast<float>(value) * (2 * maximum) / static_cast<float>((1u << bits) - 1) - maximum;
}

static uint16_t encode(const float value, const float maximum) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * 4095 / (2 * maximum));
}

void DmMotor::send() {
    if (!is_enable) {
        send_disable();
        return;
    }

    // 使能帧与 MIT 帧互斥，避免同一控制周期发送两帧
    if (enable_timer_.elapsed() >= 1.0f / enable_frequency_) {
        enable_timer_.restart();
        send_enable();
        return;
    }

    send_mit();
}

DmMotor::DmMotor(const Config& config)
    : Motor(config.motor), protocol_(config.protocol), enable_frequency_(config.enable_frequency) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void DmMotor::can_callback(const CanFrame& frame) {
    if (frame.id != config_.bus.rx_id || frame.size != 8 || (frame.data[0] & 0x0f) != (config_.bus.tx_id & 0x0f))
        return;
    const auto* data = frame.data;
    err_code = static_cast<ErrCode>(data[0] >> 4);
    temperature.mos = static_cast<int8_t>(data[6]);
    temperature.motor = static_cast<int8_t>(data[7]);

    raw.torque = decode(((data[4] & 0x0f) << 8) | data[5], config_.drive.max_torque, 12);
    raw.current = raw.torque / config_.drive.kt;
    raw.speed = decode((data[3] << 4) | (data[4] >> 4), protocol_.max_speed, 12) / config_.drive.reduction;
    raw.position = decode((data[1] << 8) | data[2], protocol_.max_position, 16);
    Motor::can_callback();
}

void DmMotor::send_mit() const {
    const float dir = config_.install.is_invert ? -1.0f : 1.0f;
    const uint16_t command = encode(is_connect && is_enable ? dir * torque.ref : 0, config_.drive.max_torque);

    // 对称量程下 p、v 为零的量化编码固定，kp、kd 为零，仅使用转矩前馈
    config_.bus.can.send({
        .id = config_.bus.tx_id,
        .size = 8,
        .data = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

void DmMotor::send_enable() const {
    config_.bus.can.send({.id = config_.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}});
}

void DmMotor::send_disable() const {
    config_.bus.can.send({.id = config_.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}});
}
