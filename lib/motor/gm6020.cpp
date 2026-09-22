#include "gm6020.hpp"
#include "math/math.hpp"
#include "math/unit.hpp"

GM6020::GM6020(const Config& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0.0f)
        this->config.reduction = REDUCTION;
    if (this->config.kt == 0.0f)
        this->config.kt = KT;
    if (this->config.r == 0.0f)
        this->config.r = R;

    // 注册CAN回调
    config.can.add_rx_callback([this](const CanFrame& frame) {
        this->callback(frame.id, frame.data.data(), frame.size);
    });
}

int16_t GM6020::get_can_cmd() const {
    if (is_connect == false || is_enable == false)
        return 0;

    int16_t cmd;

    const float current_limit = clamp(current.ref, MAX_CURRENT);
    if (!config.is_invert) {
        cmd = (int16_t)((current_limit / MAX_CURRENT) * MAX_VOLTAGE_CMD);
    } else {
        cmd = (int16_t)((-current_limit / MAX_CURRENT) * MAX_VOLTAGE_CMD);
    }

    return cmd;
}

void GM6020::callback(const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // ID、长度校验
    if (id != config.master_id)
        return;
    if (dlc != 8)
        return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto speed_i16 = (int16_t)((data[2] << 8) | data[3]);
    const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);
    temperature.motor = (int8_t)data[6];

    // 单位标准化
    const Raw raw = {
        .current = (float)current_i16 / MAX_CURRENT_CMD * MAX_CURRENT,
        .torque = raw.current * config.kt,
        .speed = (float)speed_i16 / config.reduction * rpm,
        .angle = (float)angle_u16 / 8192.0f * rev};

    // 调用父类公共回调函数
    Motor::callback(raw);
}