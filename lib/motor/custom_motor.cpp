#include "custom_motor.hpp"
#include "math/conversions.hpp"

using math::deg;
using math::deg_s;

CustomMotor::CustomMotor(const Config& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0)
        this->config.reduction = 1;

    // 注册CAN回调
    rmpp::board.can(config.can_port).add_rx_callback([this](const rmpp::CanFrame& frame) {
        this->callback(this->config.can_port, frame.id, frame.data.data(), frame.size);
    });
}

int16_t CustomMotor::get_can_cmd() const {
    if (is_enable == false)
        return 0;

    int16_t cmd;

    const int16_t current_i16 = current.ref * 100; // 0.01A/LSB
    if (!config.is_invert) {
        cmd = current_i16;
    } else {
        cmd = -current_i16;
    }

    return cmd;
}

void CustomMotor::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port)
        return;
    if (id != config.master_id)
        return;
    if (dlc != 8)
        return;

    // 解析CAN报文
    const auto angle_u16 = (uint16_t)((data[0] << 8) | data[1]);  // 0.01°/LSB，数值范围0~35999
    const auto speed_i16 = (int16_t)((data[2] << 8) | data[3]);   // 1dps/LSB
    const auto current_i16 = (int16_t)((data[4] << 8) | data[5]); // 0.01A/LSB
    temperature.motor = (int8_t)data[6];

    // 单位标准化
    const Raw raw = {
        .current = (float)current_i16 / 100.0f,
        .torque = raw.current * config.kt,
        .speed = speed_i16 * deg_s,
        .angle = (float)angle_u16 / 100.0f * deg,
    };

    // 调用父类公共回调函数
    Motor::callback(raw);
}