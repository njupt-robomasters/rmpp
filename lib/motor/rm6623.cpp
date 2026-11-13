#include "rm6623.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"

RM6623::RM6623(const Config& config)
    : Motor(config, {.max_current = MAX_CURRENT, .max_torque = MAX_TORQUE, .reduction = REDUCTION}) {
    config.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void RM6623::write_can_frame(CanFrame& frame) const {
    // 计算can偏移量
    uint32_t first_id;
    if (frame.id == 0x1ff)
        first_id = 0x205;
    else if (frame.id == 0x2ff)
        first_id = 0x209;
    else
        return;
    if (frame.size != 8 || config.bus.rx_id < first_id || config.bus.rx_id >= first_id + 4)
        return;
    const uint8_t offset = static_cast<uint8_t>(config.bus.rx_id - first_id) * 2;

    // 计算can命令
    float current = 0;
    if (is_connect && is_enable_)
        current = config.install.is_invert ? -current_ref : current_ref;
    const int16_t command = static_cast<int16_t>(current / MAX_CURRENT * 5000.0f);

    // 构造can报文
    frame.data[offset] = static_cast<uint8_t>(command >> 8);
    frame.data[offset + 1] = static_cast<uint8_t>(command);
}

void RM6623::send_calibrate_cmd() const {
    config.bus.can.send({.id = 0x3f0, .size = 8, .data = {'c'}});
}

void RM6623::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != config.bus.rx_id || frame.size != 8)
        return;

    // 解析can报文
    const auto* data = frame.data;
    // 解析电流和位置反馈，电流位于第 2、3 字节
    raw.current = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]) / 5000.0f * MAX_CURRENT;
    raw.torque = raw.current / config.drive.max_current * config.drive.max_torque;
    raw.position = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]) / 8192.0f * rev;

    const float dt = speed_timer_.restart();
    raw.speed = has_position_ && dt > 0 ? static_cast<float>(Angle(raw.position - position_last_)) / dt : 0;
    position_last_ = raw.position;
    has_position_ = true;

    // 调用上级回调
    Motor::can_callback();
}
