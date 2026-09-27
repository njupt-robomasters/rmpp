#include "m3508.hpp"

M3508::M3508(const Config& config, Protocol protocol)
    : DjiMotor(apply_defaults(config, DEFAULT_CONFIG),
               {.feedback_scale = protocol.feedback_scale ? protocol.feedback_scale : DEFAULT_PROTOCOL.feedback_scale,
                .command_scale = protocol.command_scale ? protocol.command_scale : DEFAULT_PROTOCOL.command_scale}) {
    subscribe();
}

void M3508::parse_feedback(const uint8_t data[8]) {
    temperature.motor = static_cast<int8_t>(data[6]);
    DjiMotor::parse_feedback(data);
}

bool M3508::send_quick_set_id() const {
    return config.protocol.can.send({.id = 0x700, .size = 8});
}
