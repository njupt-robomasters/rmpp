#include "mf9025.hpp"
#include "math/math.hpp"
#include "math/unit.hpp"

MF9025::MF9025(const Config& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0.0f)
        this->config.reduction = REDUCTION;
    if (this->config.kt == 0.0f)
        this->config.kt = KT;

    // 注册CAN回调
    config.can.add_rx_callback([this](const CanFrame& frame) {
        this->callback(frame.id, frame.data.data(), frame.size);
    });
}

void MF9025::send_can_cmd() {
    send_cnt_++;
    if (is_connect && is_enable) {
        send_current(current.ref);
    } else {
        send_current(0);
    }
}

void MF9025::callback(const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // ID、长度校验
    if (id != config.master_id)
        return;
    if (dlc != 8)
        return;

    // 命令码校验
    if (data[0] != 0xA1)
        return;

    // 解析CAN报文
    temperature.motor = (int8_t)data[1];
    const auto current_i16 = (int16_t)(data[2] | (data[3] << 8));
    const auto speed_i16 = (int16_t)(data[4] | (data[5] << 8));
    const auto angle_u16 = (uint16_t)(data[6] | (data[7] << 8));

    // 单位标准化
    const Raw raw = {
        .current = (float)current_i16 / MAX_CURRENT_CMD * MAX_CURRENT,
        .torque = raw.current * config.kt,
        .speed = speed_i16 * deg_s,
        .angle = (float)angle_u16 / 65535.0f * rev,
    };

    // 调用父类公共回调函数
    Motor::callback(raw);
}

void MF9025::send_current(float current) const {
    int16_t cmd;

    const float current_limit = clamp(current, MAX_CURRENT);
    if (!config.is_invert) {
        cmd = (int16_t)((current_limit / MAX_CURRENT) * MAX_CURRENT_CMD);
    } else {
        cmd = (int16_t)((-current_limit / MAX_CURRENT) * MAX_CURRENT_CMD);
    }

    std::array<uint8_t, 8> data{};
    data[0] = 0xA1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = cmd;
    data[5] = cmd >> 8;
    data[6] = 0;
    data[7] = 0;
    config.can.send({.id = config.slave_id, .data = data, .size = 8});
}