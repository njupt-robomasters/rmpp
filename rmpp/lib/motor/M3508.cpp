#include "M3508.hpp"

M3508::M3508(const config_t& config) : Motor(config) {
    // 注册CAN回调
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

int16_t M3508::GetCanCmd() const {
    if (is_enable == false) return 0;

    int16_t cmd;

    const UnitFloat current_limit = unit::clamp(current.ref, MAX_CURRENT);
    if (!config.is_invert) {
        cmd = (int16_t)((current_limit / MAX_CURRENT).toFloat(A) * MAX_CURRENT_CMD);
    } else {
        cmd = (int16_t)((-current_limit / MAX_CURRENT).toFloat(A) * MAX_CURRENT_CMD);
    }

    return cmd;
}

void M3508::SendQuickSetID() const {
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    BSP::CAN::TransmitStd(config.can_port, 0x700, data, 8);
}

void M3508::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port) return;
    if (id != config.master_id) return;
    if (dlc != 8) return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto speed_i16 = (int16_t)((data[2] << 8) | data[3]);
    const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);
    temperature.motor = (int8_t)data[6] * C;

    // 单位标准化
    const raw_t raw = {
        .current = (float)current_i16 / (float)MAX_CURRENT_CMD * MAX_CURRENT,
        .torque = raw.current * config.Kt,
        .speed = (float)speed_i16 / config.reduction * rpm,
        .angle = (float)angle_u16 / 8192.0f * rev
    };

    // 调用父类公共回调函数
    Motor::callback(raw);
}