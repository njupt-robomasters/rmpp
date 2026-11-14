#pragma once

#include <algorithm>

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send() {
    if (!is_enable) {
        send_disable();
        return;
    }

    if (err_code != NONE && err_code != ENABLED) {
        send_clear_err();
        return;
    }

    if (enable_timer.elapsed() >= 1.0f / ENABLE_FREQUENCY) {
        enable_timer.restart();
        send_enable();
        return;
    }

    send_mit();
}

template <float PMAX, float VMAX, float TMAX>
DmMotor<PMAX, VMAX, TMAX>::DmMotor(const Config& config, const float max_current, const float max_torque, const float reduction)
    : Motor(config, max_current, max_torque, reduction) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::can_callback(const CanFrame& frame) {
    if (frame.id != rx_id || frame.size != 8 || (frame.data[0] & 0x0f) != (tx_id & 0x0f))
        return;

    const auto* data = frame.data;
    err_code = static_cast<ErrCode>(data[0] >> 4);
    temp_mos = static_cast<int8_t>(data[6]);
    temp_motor = static_cast<int8_t>(data[7]);

    raw_torque = decode(((data[4] & 0x0f) << 8) | data[5], TMAX, 12);
    raw_current = raw_torque / max_torque * max_current;
    raw_speed = decode((data[3] << 4) | (data[4] >> 4), VMAX, 12);
    raw_position = decode((data[1] << 8) | data[2], PMAX, 16);
    Motor::can_callback();
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_mit() const {
    const float dir = is_invert ? -1.0f : 1.0f;
    const uint16_t command = encode(is_connect && is_enable ? dir * torque_ref : 0, TMAX, 12);

    // 对称量程下 p、v 为零的量化编码固定，kp、kd 为零，仅使用转矩前馈
    can.send({
        .id = tx_id,
        .size = 8,
        .data = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_enable() const {
    can.send({.id = tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}});
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_disable() const {
    can.send({.id = tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}});
}

template <float PMAX, float VMAX, float TMAX>
void DmMotor<PMAX, VMAX, TMAX>::send_clear_err() const {
    can.send({.id = tx_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb}});
}

template <float PMAX, float VMAX, float TMAX>
float DmMotor<PMAX, VMAX, TMAX>::decode(const uint16_t value, const float maximum, const unsigned bits) {
    return static_cast<float>(value) * (2 * maximum) / static_cast<float>((1u << bits) - 1) - maximum;
}

template <float PMAX, float VMAX, float TMAX>
uint16_t DmMotor<PMAX, VMAX, TMAX>::encode(const float value, const float maximum, const unsigned bits) {
    return static_cast<uint16_t>((std::clamp(value, -maximum, maximum) + maximum) * ((1u << bits) - 1) / (2 * maximum));
}
