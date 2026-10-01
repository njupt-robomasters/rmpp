#include "c620.hpp"

C620::C620(const Config& config)
    : DjiMotor(config.with_defaults(DRIVE)) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

bool C620::write_can_frame(CanFrame& frame) const {
    const int16_t command = static_cast<int16_t>(directed_current() / config_.drive.max_current * 16384);
    if (frame.id == 0x200)
        return write_command(frame, command, 0x201);
    if (frame.id == 0x1ff)
        return write_command(frame, command, 0x205);
    return false;
}

void C620::send_quick_set_id() const {
    config_.bus.can.send({.id = 0x700, .size = 8});
}

void C620::can_callback(const CanFrame& frame) {
    if (frame.id != config_.bus.rx_id || frame.size != 8)
        return;
    const auto* data = frame.data;
    temperature = static_cast<int8_t>(data[6]);
    const float current = static_cast<int16_t>((data[4] << 8) | data[5]) / 16384.0f * config_.drive.max_current;
    DjiMotor::can_callback((data[0] << 8) | data[1], current, static_cast<int16_t>((data[2] << 8) | data[3]));
}
