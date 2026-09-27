#pragma once

#include "dji_motor.hpp"

class RM6623 : public DjiMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 0.38f, .max_current = 5},
        .install = {.reduction = 1},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.feedback_scale = 5000, .command_scale = 5000};

    explicit RM6623(const Config& config, Protocol protocol = {});

    bool send_calibrate_cmd() const;

  private:
    void parse_feedback(const uint8_t data[8]) override;
};
