#pragma once

#include "dji_motor.hpp"

class M2006 : public DjiMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 0.18f, .max_current = 10},
        .install = {.reduction = 36.0f},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.feedback_scale = 10000, .command_scale = 10000};

    explicit M2006(const Config& config, Protocol protocol = {});
};
