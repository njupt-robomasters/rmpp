#include "rm6623.hpp"

RM6623::RM6623(const Config& config)
    : DjiMotor(config.with_defaults(DRIVE)) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

bool RM6623::write_can_frame(CanFrame& frame) const {
    const int16_t command = static_cast<int16_t>(directed_current() / config_.drive.max_current * 5000);
    if (frame.id == 0x1ff)
        return write_command(frame, command, 0x205);
    if (frame.id == 0x2ff)
        return write_command(frame, command, 0x209);
    return false;
}

void RM6623::send_calibrate_cmd() const {
    config_.bus.can.send({.id = 0x3f0, .size = 8, .data = {'c'}});
}

void RM6623::can_callback(const CanFrame& frame) {
    if (frame.id != config_.bus.rx_id || frame.size != 8)
        return;
    const auto* data = frame.data;
    // 没有速度反馈，电流位于第 2、3 字节
    const float current = static_cast<int16_t>((data[2] << 8) | data[3]) / 5000.0f * config_.drive.max_current;
    DjiMotor::can_callback((data[0] << 8) | data[1], current, 0);
}
