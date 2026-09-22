#include "rm6623.hpp"
#include "math/math.hpp"
#include "math/unit.hpp"

RM6623::RM6623(const Config& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0)
        this->config.reduction = REDUCTION;
    if (this->config.kt == 0)
        this->config.kt = KT;

    // 注册CAN回调
    config.can.add_rx_callback([this](const CanFrame& frame) {
        this->callback(frame.id, frame.data.data(), frame.size);
    });
}

int16_t RM6623::get_can_cmd() const {
    if (is_connect == false || is_enable == false)
        return 0;

    int16_t cmd;

    const float current_limit = clamp(current.ref, MAX_CURRENT);
    if (!config.is_invert) {
        cmd = (int16_t)((current_limit / MAX_CURRENT) * MAX_CURRENT_CMD);
    } else {
        cmd = (int16_t)((-current_limit / MAX_CURRENT) * MAX_CURRENT_CMD);
    }

    return cmd;
}

void RM6623::send_calibrate_cmd() const {
    std::array<uint8_t, 8> data = {'c', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    config.can.send({.id = 0x3F0, .data = data, .size = 8}); // 校准指令固定ID=0x3F0
}

void RM6623::callback(const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // ID、长度校验
    if (id != config.master_id)
        return;
    if (dlc != 8)
        return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const auto current_measure_i16 = (int16_t)((data[2] << 8) | data[3]);
    // const auto current_ref_i16 = (int16_t)((data[4] << 8) | data[5]);

    // 单位标准化
    const Raw raw = {
        .current = (float)current_measure_i16 / MAX_CURRENT_CMD * MAX_CURRENT,
        .torque = raw.current * config.kt,
        .angle = (float)angle_u16 / 8192.0f * rev};

    // 调用父类公共回调函数
    Motor::callback(raw);
}