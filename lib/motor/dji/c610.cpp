#include "c610.hpp"

C610::C610(const Config& config)
    : DjiMotor(config.with_defaults(DRIVE)) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

bool C610::write_can_frame(CanFrame& frame) const {
    const int16_t command = static_cast<int16_t>(directed_current() / config_.drive.max_current * 10000);
    if (frame.id == 0x200)
        return write_command(frame, command, 0x201);
    if (frame.id == 0x1ff)
        return write_command(frame, command, 0x205);
    return false;
}

void C610::can_callback(const CanFrame& frame) {
    if (frame.id != config_.bus.rx_id || frame.size != 8)
        return;
    const auto* data = frame.data;
    const float current = static_cast<int16_t>((data[4] << 8) | data[5]) / 10000.0f * config_.drive.max_current;
    DjiMotor::can_callback((data[0] << 8) | data[1], current, static_cast<int16_t>((data[2] << 8) | data[3]));
}
