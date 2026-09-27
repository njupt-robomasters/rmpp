#include "dm6006.hpp"

DM6006::DM6006(const Config& config, Protocol protocol)
    : DmMotor(apply_defaults(config, DEFAULT_CONFIG),
              {.max_speed = protocol.max_speed ? protocol.max_speed : DEFAULT_PROTOCOL.max_speed,
               .max_position = protocol.max_position ? protocol.max_position : DEFAULT_PROTOCOL.max_position}) {
    subscribe();
}
