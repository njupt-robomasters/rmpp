#include "gm6020.hpp"
#include "math/unit.hpp"
#include <algorithm>

GM6020::GM6020(const Config& config, const float ke)
    : Motor(config, MAX_CURRENT, MAX_TORQUE, REDUCTION),
      ke(ke) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void GM6020::write_can_frame(CanFrame& frame) const {
    // 计算can偏移量
    uint32_t first_id;
    uint8_t motor_count;
    if (frame.id == 0x1ff || frame.id == 0x1fe) {
        first_id = 0x205;
        motor_count = 4;
    } else if (frame.id == 0x2ff || frame.id == 0x2fe) {
        first_id = 0x209;
        motor_count = 3;
    } else {
        return;
    }
    if (frame.size != 8 || rx_id < first_id || rx_id >= first_id + motor_count)
        return;
    const uint8_t offset = static_cast<uint8_t>(rx_id - first_id) * 2;

    // 计算can命令，先在安装方向修正后的坐标系计算，再换回电机方向
    const float dir = is_invert ? -1.0f : 1.0f;
    int16_t command = 0;
    if (frame.id == 0x1ff || frame.id == 0x2ff) { // 电压模式
        float voltage = 0;
        if (is_connect && is_enable)
            voltage = std::clamp(current_ref / MAX_CURRENT * MAX_VOLTAGE + ke * speed, -MAX_VOLTAGE, MAX_VOLTAGE);
        command = static_cast<int16_t>(dir * voltage / MAX_VOLTAGE * 25000.0f);
    } else { // 电流模式
        float current = 0;
        if (is_connect && is_enable)
            current = current_ref;
        command = static_cast<int16_t>(dir * current / MAX_CURRENT * 16384.0f);
    }

    // 构造can报文
    frame.data[offset] = static_cast<uint8_t>(command >> 8);
    frame.data[offset + 1] = static_cast<uint8_t>(command);
}

void GM6020::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != rx_id || frame.size != 8)
        return;

    // 解析can报文
    const auto* data = frame.data;
    raw_position = static_cast<float>((data[0] << 8) | data[1]) / 8192.0f * rev;
    raw_speed = static_cast<int16_t>((data[2] << 8) | data[3]) * rpm;
    raw_current = static_cast<int16_t>((data[4] << 8) | data[5]) / 16384.0f * MAX_CURRENT;
    motor_temperature = static_cast<int8_t>(data[6]);
    raw_torque = raw_current / max_current * max_torque;

    // 调用上级回调
    Motor::can_callback();
}
