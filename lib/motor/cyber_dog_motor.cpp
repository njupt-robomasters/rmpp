#include "cyber_dog_motor.hpp"
#include <algorithm>

static float decode(const uint16_t value, const float maximum, const unsigned bits) {
    return static_cast<float>(value) * (2 * maximum) / static_cast<float>((1u << bits) - 1) - maximum;
}

static uint16_t encode(const float value, const float maximum) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * 4095 / (2 * maximum));
}

CyberDogMotor::CyberDogMotor(const Config& config)
    : Motor(config.with_defaults(DRIVE)) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void CyberDogMotor::send() {
    send_count_++;
    if (!is_connect || !is_enable)
        send_disable();
    else if (send_count_ % 100 == 0) // 每 100 次调用重新发送使能
        send_enable();
    else
        send_mit();
}

void CyberDogMotor::can_callback(const CanFrame& frame) {
    if (frame.id != config_.bus.rx_id || frame.size != 8 || frame.data[0] != config_.bus.tx_id)
        return;

    const auto* data = frame.data;
    raw.current = decode(((data[4] & 0x0f) << 8) | data[5], config_.drive.max_current, 12);
    raw.torque = raw.current * config_.drive.kt;
    raw.speed = decode((data[3] << 4) | (data[4] >> 4), MAX_SPEED, 12) / config_.drive.reduction;
    raw.position = decode((data[1] << 8) | data[2], MAX_POSITION, 16);
    Motor::can_callback();
}

void CyberDogMotor::send_mit() const {
    const float dir = config_.install.is_invert ? -1.0f : 1.0f;
    const uint16_t command = encode(is_connect && is_enable ? dir * current.ref : 0, config_.drive.max_current);

    // 对称量程下 p、v 为零的量化编码固定，kp、kd 为零，仅使用电流前馈
    config_.bus.can.send({
        .id = config_.bus.tx_id,
        .size = 8,
        .data = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

void CyberDogMotor::send_enable() const {
    config_.bus.can.send({.id = config_.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}});
}

void CyberDogMotor::send_disable() const {
    config_.bus.can.send({.id = config_.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}});
}
