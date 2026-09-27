#include "dji_motor.hpp"

DjiMotor::DjiMotor(const Config& config, Protocol protocol)
    : Motor(config), protocol(protocol) {}

int16_t DjiMotor::command_value() const {
    return static_cast<int16_t>(output_current(config.physical.max_current) / config.physical.max_current * protocol.command_scale);
}

void DjiMotor::receive(const CanFrame& frame) {
    if (is_feedback_frame(frame))
        parse_feedback(frame.data);
}

void DjiMotor::parse_feedback(const uint8_t data[8]) {
    const auto encoder = static_cast<uint16_t>((data[0] << 8) | data[1]);
    const auto speed = static_cast<int16_t>((data[2] << 8) | data[3]);
    const auto current = static_cast<int16_t>((data[4] << 8) | data[5]);
    accept_dji_feedback(encoder, current, speed);
}

void DjiMotor::accept_dji_feedback(uint16_t encoder, int16_t current, int16_t speed) {
    const float amperes = float(current) / protocol.feedback_scale * config.physical.max_current;
    accept_feedback({
        .current = amperes,
        .torque = amperes * config.physical.kt,
        .speed = float(speed) / config.install.reduction * rpm,
        .position = float(encoder) / 8192 * rev,
    });
}
