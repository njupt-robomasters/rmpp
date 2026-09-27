#pragma once

#include "dji_motor.hpp"

class GM6020 : public DjiMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 0.741f, .max_current = 3},
        .install = {.reduction = 1},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.feedback_scale = 16384, .command_scale = 25000};

    explicit GM6020(const Config& config, Protocol protocol = {});

  private:
    void parse_feedback(const uint8_t data[8]) override;
};
