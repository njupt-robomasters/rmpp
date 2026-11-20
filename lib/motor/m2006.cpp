#include "m2006.hpp"
#include "math/unit.hpp"

M2006::M2006(const Config& config)
    : Motor(config, MAX_CURRENT, MAX_TORQUE, REDUCTION) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void M2006::write_can_frame(CanFrame& frame) const {
    // 计算can偏移量
    uint32_t first_id;
    if (frame.id == 0x200)
        first_id = 0x201;
    else if (frame.id == 0x1ff)
        first_id = 0x205;
    else
        return;
    if (frame.size != 8 || config.rx_id < first_id || config.rx_id >= first_id + 4)
        return;
    const uint8_t offset = static_cast<uint8_t>(config.rx_id - first_id) * 2;

    // 计算can命令
    float current = 0;
    if (is_connect && is_enable)
        current = config.is_invert ? -current_ref : current_ref;
    const auto command = static_cast<int16_t>(current / MAX_CURRENT * 10000.0f);

    // 构造can报文
    frame.data[offset] = static_cast<uint8_t>(command >> 8);
    frame.data[offset + 1] = static_cast<uint8_t>(command);
}

void M2006::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != config.rx_id || frame.size != 8)
        return;

    // 解析can报文并调用上级回调
    const auto* data = frame.data;
    Motor::can_callback({
        .current = static_cast<int16_t>((data[4] << 8) | data[5]) / 10000.0f * MAX_CURRENT,
        .speed = static_cast<int16_t>((data[2] << 8) | data[3]) * rpm,
        .angle = static_cast<float>((data[0] << 8) | data[1]) / 8192.0f * rev,
    });
}
