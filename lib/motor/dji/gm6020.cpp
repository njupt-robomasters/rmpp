#include "gm6020.hpp"

GM6020::GM6020(const Config& config, Protocol protocol)
    : DjiMotor(apply_defaults(config, DEFAULT_CONFIG),
               {.feedback_scale = protocol.feedback_scale ? protocol.feedback_scale : DEFAULT_PROTOCOL.feedback_scale,
                .command_scale = protocol.command_scale ? protocol.command_scale : DEFAULT_PROTOCOL.command_scale}) {
    subscribe();
}

void GM6020::parse_feedback(const uint8_t data[8]) {
    temperature.motor = static_cast<int8_t>(data[6]);
    DjiMotor::parse_feedback(data);
}
