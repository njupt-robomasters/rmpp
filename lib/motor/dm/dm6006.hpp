#pragma once

#include "dm_motor.hpp"

class DM6006 : public DmMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 1, .max_torque = 12},
        .install = {.reduction = 1},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.max_speed = 45, .max_position = 3.14159f};

    explicit DM6006(const Config& config, Protocol protocol = {});
};
