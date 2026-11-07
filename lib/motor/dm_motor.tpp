#pragma once

#include <algorithm>

template <float PMAX, float VMAX, float TMAX>
float DmMotor<PMAX, VMAX, TMAX>::decode(const uint16_t value, const float maximum, const unsigned bits) {
    return static_cast<float>(value) * (2 * maximum) / static_cast<float>((1u << bits) - 1) - maximum;
}

template <float PMAX, float VMAX, float TMAX>
uint16_t DmMotor<PMAX, VMAX, TMAX>::encode(const float value, const float maximum, const unsigned bits) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * ((1u << bits) - 1) / (2 * maximum));
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send() {
    if (!is_enable_) {
        send_disable();
        return;
    }

    if (err_code != NONE && err_code != ENABLED) {
        send_clear_err();
        return;
    }

    if (enable_timer_.elapsed() >= 1.0f / ENABLE_FREQUENCY) {
        enable_timer_.restart();
        send_enable();
        return;
    }

    send_mit();
}

template <float PMAX, float VMAX, float TMAX>
DmMotor<PMAX, VMAX, TMAX>::DmMotor(const Motor::Config& config)
    : Motor(config) {
    config.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::can_callback(const CanFrame& frame) {
    if (frame.id != config.bus.rx_id || frame.size != 8 || (frame.data[0] & 0x0f) != (config.bus.tx_id & 0x0f))
        return;

    const auto* data = frame.data;
    err_code = static_cast<ErrCode>(data[0] >> 4);
    temp_mos = static_cast<int8_t>(data[6]);
    temp_motor = static_cast<int8_t>(data[7]);

    raw.torque = decode(((data[4] & 0x0f) << 8) | data[5], TMAX, 12);
    raw.current = raw.torque / config.drive.kt;
    raw.speed = decode((data[3] << 4) | (data[4] >> 4), VMAX, 12) / config.drive.reduction;
    raw.position = decode((data[1] << 8) | data[2], PMAX, 16);
    Motor::can_callback();
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_mit() const {
    const float dir = config.install.is_invert ? -1.0f : 1.0f;
    const uint16_t command = encode(is_connect && is_enable_ ? dir * torque_ref : 0, TMAX, 12);

    // 对称量程下 p、v 为零的量化编码固定，kp、kd 为零，仅使用转矩前馈
    config.bus.can.send({
        .id = config.bus.tx_id,
        .size = 8,
        .data = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_enable() const {
    config.bus.can.send({.id = config.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}});
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_disable() const {
    config.bus.can.send({.id = config.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}});
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_clear_err() const {
    config.bus.can.send({.id = config.bus.tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb}});
}
