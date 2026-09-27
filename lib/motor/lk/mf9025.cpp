#include "mf9025.hpp"

MF9025::MF9025(const Config& config)
    : MF9025(config, {}) {}

MF9025::MF9025(const Config& config, Protocol protocol)
    : Motor(apply_defaults(config, DEFAULT_CONFIG)),
      protocol{.feedback_scale = protocol.feedback_scale ? protocol.feedback_scale : DEFAULT_PROTOCOL.feedback_scale,
               .command_scale = protocol.command_scale ? protocol.command_scale : DEFAULT_PROTOCOL.command_scale} {
    subscribe();
}

void MF9025::receive(const CanFrame& frame) {
    if (!is_feedback_frame(frame) || frame.data[0] != 0xa1)
        return;
    const auto* data = frame.data;
    temperature.motor = static_cast<int8_t>(data[1]);
    const auto current = static_cast<int16_t>(data[2] | (data[3] << 8));
    const auto speed = static_cast<int16_t>(data[4] | (data[5] << 8));
    const auto encoder = static_cast<uint16_t>(data[6] | (data[7] << 8));
    const float amperes = float(current) / this->protocol.feedback_scale * config.physical.max_current;
    accept_feedback({
        .current = amperes,
        .torque = amperes * config.physical.kt,
        .speed = speed * deg_s / config.install.reduction,
        .position = float(encoder) / 65535 * rev,
    });
}

bool MF9025::send_can_cmd() {
    update_connection();
    const auto command = static_cast<int16_t>(output_current(config.physical.max_current) / config.physical.max_current * protocol.command_scale);
    return config.protocol.can.send({
        .id = config.protocol.slave_id,
        .size = 8,
        .data = {0xa1, 0, 0, 0, static_cast<uint8_t>(command), static_cast<uint8_t>(command >> 8), 0, 0},
    });
}
