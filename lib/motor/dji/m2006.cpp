#include "m2006.hpp"

M2006::M2006(const Config& config, Protocol protocol)
    : DjiMotor(apply_defaults(config, DEFAULT_CONFIG),
               {.feedback_scale = protocol.feedback_scale ? protocol.feedback_scale : DEFAULT_PROTOCOL.feedback_scale,
                .command_scale = protocol.command_scale ? protocol.command_scale : DEFAULT_PROTOCOL.command_scale}) {
    subscribe();
}
