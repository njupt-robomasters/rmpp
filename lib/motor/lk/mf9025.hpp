#pragma once

#include "motor/motor.hpp"

class MF9025 : public Motor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 0.32f, .max_current = 16.5f},
        .install = {.reduction = 1},
    };

    struct Protocol {
        int16_t feedback_scale = 0, command_scale = 0; // 电流反馈、命令满量程刻度
    } const protocol;

    static constexpr Protocol DEFAULT_PROTOCOL{.feedback_scale = 2048, .command_scale = 2048};
    explicit MF9025(const Config& config);
    MF9025(const Config& config, Protocol protocol);
    bool send_can_cmd();

  private:
    void receive(const CanFrame& frame) override;
};
