#include "MF9025.hpp"

MF9025::MF9025(const config_t& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0.0f) this->config.reduction = REDUCTION;
    if (this->config.Kt == 0.0f) this->config.Kt = Kt;

    // 注册CAN回调
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

void MF9025::SendCanCmd() {
    send_cnt++;
    if (is_connect && is_enable) {
        sendCurrent(current.ref);
    } else {
        sendCurrent(0 * A);
    }
}

void MF9025::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port) return;
    if (id != config.master_id) return;
    if (dlc != 8) return;

    // 命令码校验
    if (data[0] != 0xA1) return;

    // 解析CAN报文
    temperature.motor = (int8_t)data[1] * C;
    const auto current_i16 = (int16_t)(data[2] | (data[3] << 8));
    const auto speed_i16 = (int16_t)(data[4] | (data[5] << 8));
    const auto angle_u16 = (uint16_t)(data[6] | (data[7] << 8));

    // 单位标准化
    const raw_t raw = {
        .current = (float)current_i16 / MAX_CURRENT_CMD * MAX_CURRENT,
        .torque = raw.current * config.Kt,
        .speed = speed_i16 * deg_s,
        .angle = (float)angle_u16 / 65535.0f * rev,
    };

    // 调用父类公共回调函数
    Motor::callback(raw);
}

void MF9025::sendCurrent(const UnitFloat<>& current) const {
    int16_t cmd;

    const UnitFloat current_limit = unit::clamp(current, MAX_CURRENT);
    if (!config.is_invert) {
        cmd = (int16_t)((current_limit / MAX_CURRENT).toFloat(A) * MAX_CURRENT_CMD);
    } else {
        cmd = (int16_t)((-current_limit / MAX_CURRENT).toFloat(A) * MAX_CURRENT_CMD);
    }

    uint8_t data[8];
    data[0] = 0xA1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = cmd;
    data[5] = cmd >> 8;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(config.can_port, config.slave_id, data);
}