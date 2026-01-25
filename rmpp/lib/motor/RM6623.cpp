#include "RM6623.hpp"

RM6623::RM6623(const config_t& config) : Motor(config) {
    // 注册CAN回调
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

int16_t RM6623::GetCurrentCmd() const {
    if (is_connect == false || is_enable == false) return 0;

    int16_t current_cmd;
    if (!config.is_invert) {
        current_cmd = (int16_t)(current.ref.toFloat(A) * 1000.0f);
    } else {
        current_cmd = (int16_t)((-current.ref).toFloat(A) * 1000.0f);
    }
    return current_cmd;
}

void RM6623::SendCalibrateCmd() const {
    uint8_t data[8] = {'c', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(config.can_port, 0x3F0, data, 8); // 校准指令固定ID=0x3F0
}

void RM6623::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port) return;
    if (id != config.master_id) return;
    if (dlc != 8) return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto current_measure_i16 = (int16_t)((data[2] << 8) | data[3]);
    // const auto current_ref_i16 = (int16_t)((data[4] << 8) | data[5]);

    // 单位标准化
    const raw_t raw = {
        .current = (float)current_measure_i16 / 1000.0f * A,
        .torque = raw.current * config.Kt,
        .angle = (float)angle_u16 / 8192.0f * rev
    };

    // 调用父类公共回调函数
    Motor::callback(raw);
}