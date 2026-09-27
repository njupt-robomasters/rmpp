#include "mit_motor.hpp"

static float decode(uint16_t value, float maximum, unsigned bits) {
    return float(value) * (2 * maximum) / ((1u << bits) - 1) - maximum;
}

static uint16_t encode(float value, float maximum) {
    return static_cast<uint16_t>((value + maximum) * 4095 / (2 * maximum));
}

MitMotor::MitMotor(const Config& config, Protocol protocol)
    : Motor(config), protocol(protocol) {}

void MitMotor::accept_mit_feedback(const uint8_t data[8], PidOutputType output_type) {
    const bool current_output = output_type == CURRENT_OUTPUT;
    const float value = decode(((data[4] & 0x0f) << 8) | data[5], current_output ? config.physical.max_current : config.physical.max_torque, 12);
    accept_feedback({
        .current = current_output ? value : value / config.physical.kt,
        .torque = current_output ? value * config.physical.kt : value,
        .speed = decode((data[3] << 4) | (data[4] >> 4), protocol.max_speed, 12) / config.install.reduction,
        .position = decode((data[1] << 8) | data[2], protocol.max_position, 16),
    });
}

bool MitMotor::send_mit(PidOutputType output_type) const {
    const uint16_t command = output_type == CURRENT_OUTPUT
                                 ? encode(output_current(config.physical.max_current), config.physical.max_current)
                                 : encode(output_torque(config.physical.max_torque), config.physical.max_torque);
    // 对称量程下 p、v 为零的量化编码固定，kp、kd 为零，仅使用前馈
    return config.protocol.can.send({
        .id = config.protocol.slave_id,
        .size = 8,
        .data = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(command)},
    });
}

bool MitMotor::send_state(uint8_t command) const {
    return config.protocol.can.send({.id = config.protocol.slave_id, .size = 8, .data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, command}});
}
