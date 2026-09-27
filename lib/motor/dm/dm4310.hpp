#pragma once

#include "dm_motor.hpp"

class DM4310 : public DmMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 1.2f, .max_torque = 10},
        .install = {.reduction = 1},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.max_speed = 30, .max_position = 3.14159f};

    explicit DM4310(const Config& config, Protocol protocol = {});
};
