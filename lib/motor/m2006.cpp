#include "m2006.hpp"
#include "math/unit.hpp"

M2006::M2006(const Config& config)
    : Motor(config) {
    config.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
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
    if (frame.size != 8 || config.bus.rx_id < first_id || config.bus.rx_id >= first_id + 4)
        return;
    const uint8_t offset = static_cast<uint8_t>(config.bus.rx_id - first_id) * 2;

    // 计算can命令
    float current = 0;
    if (is_connect && is_enable_)
        current = config.install.is_invert ? -current_ref : current_ref;
    const auto command = static_cast<int16_t>(current / MAX_CURRENT * 10000.0f);

    // 构造can报文
    frame.data[offset] = static_cast<uint8_t>(command >> 8);
    frame.data[offset + 1] = static_cast<uint8_t>(command);
}

void M2006::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != config.bus.rx_id || frame.size != 8)
        return;

    // 解析can报文
    const auto* data = frame.data;
    raw.current = static_cast<float>((data[4] << 8) | data[5]) / 10000.0f * MAX_CURRENT;
    raw.torque = raw.current * config.drive.kt;
    raw.speed = static_cast<float>((data[2] << 8) | data[3]) * rpm;
    raw.position = static_cast<float>((data[0] << 8) | data[1]) / 8192.0f * rev;

    // 调用上级回调
    Motor::can_callback();
}
