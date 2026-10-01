#include "gm6020.hpp"

GM6020::GM6020(const Config& config)
    : DjiMotor(config.with_defaults(DRIVE)) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

bool GM6020::write_can_frame(CanFrame& frame) const {
    int16_t command;
    uint32_t first_rx_id;
    uint8_t motor_count = 4;
    if (frame.id == 0x1ff || frame.id == 0x2ff) {
        command = static_cast<int16_t>(directed_current() / config_.drive.max_current * 25000);
    } else if (frame.id == 0x1fe || frame.id == 0x2fe) {
        command = static_cast<int16_t>(directed_current() / config_.drive.max_current * 16384);
    } else {
        return false;
    }

    if (frame.id == 0x1ff || frame.id == 0x1fe) {
        first_rx_id = 0x205;
    } else {
        first_rx_id = 0x209;
        motor_count = 3;
    }
    return write_command(frame, command, first_rx_id, motor_count);
}

void GM6020::can_callback(const CanFrame& frame) {
    if (frame.id != config_.bus.rx_id || frame.size != 8)
        return;
    const auto* data = frame.data;
    temperature = static_cast<int8_t>(data[6]);
    const float current = static_cast<int16_t>((data[4] << 8) | data[5]) / 16384.0f * config_.drive.max_current;
    DjiMotor::can_callback((data[0] << 8) | data[1], current, static_cast<int16_t>((data[2] << 8) | data[3]));
}
