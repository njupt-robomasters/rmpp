#pragma once

#include "mit_motor.hpp"

class CyberDogMotor : public MitMotor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 0.9f, .max_current = 40},
        .install = {.reduction = 1},
    };
    static constexpr Protocol DEFAULT_PROTOCOL{.max_speed = 65, .max_position = 12.5f};

    explicit CyberDogMotor(const Config& config, Protocol protocol = {});
    bool send_can_cmd() override;

  private:
    uint32_t send_count_ = 0;
    void receive(const CanFrame& frame) override;
};
