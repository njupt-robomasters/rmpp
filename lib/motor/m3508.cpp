#include "m3508.hpp"
#include "math/conversions.hpp"
#include "math/math.hpp"

using math::rev;
using math::rpm;

M3508::M3508(const Config& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0)
        this->config.reduction = REDUCTION;
    if (this->config.kt == 0)
        this->config.kt = KT;
    if (this->config.r == 0)
        this->config.r = R;

    // 注册CAN回调
    rmpp::board.can(config.can_port).add_rx_callback([this](const rmpp::CanFrame& frame) {
        this->callback(this->config.can_port, frame.id, frame.data.data(), frame.size);
    });
}

int16_t M3508::get_can_cmd() const {
    if (is_enable == false)
        return 0;

    int16_t cmd;

    const float current_limit = math::clamp(current.ref, MAX_CURRENT);
    if (!config.is_invert) {
        cmd = (int16_t)((current_limit / MAX_CURRENT) * MAX_CURRENT_CMD);
    } else {
        cmd = (int16_t)((-current_limit / MAX_CURRENT) * MAX_CURRENT_CMD);
    }

    return cmd;
}

void M3508::send_quick_set_id() const {
    std::array<uint8_t, 8> data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rmpp::board.can(config.can_port).send({.id = 0x700, .data = data, .size = 8});
}

void M3508::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port)
        return;
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
        .current = (float)current_i16 / (float)MAX_CURRENT_CMD * MAX_CURRENT,
        .torque = raw.current * config.kt,
        .speed = (float)speed_i16 / config.reduction * rpm,
        .angle = (float)angle_u16 / 8192.0f * rev};

    // 调用父类公共回调函数
    Motor::callback(raw);
}