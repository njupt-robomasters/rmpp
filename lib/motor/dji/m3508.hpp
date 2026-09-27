#pragma once

#include "dji_motor.hpp"

class M3508 : public DjiMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 0.3f, .max_current = 20},
        .install = {.reduction = 3591.0f / 187.0f},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.feedback_scale = 16384, .command_scale = 16384};

    explicit M3508(const Config& config, Protocol protocol = {});

    bool send_quick_set_id() const;

  private:
    void parse_feedback(const uint8_t data[8]) override;
};
