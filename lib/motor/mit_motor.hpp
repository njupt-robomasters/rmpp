#pragma once

#include "motor.hpp"

class MitMotor : public Motor {
  public:
    struct Protocol {
        float max_speed = 0, max_position = 0; // 原始速度、位置对称量程，rad/s、rad
    } const protocol;

    virtual bool send_can_cmd() = 0;

  protected:
    MitMotor(const Config& config, Protocol protocol);
    // 共用 MIT 位布局；前馈量的单位由协议子类明确指定
    void accept_mit_feedback(const uint8_t data[8], PidOutputType output_type);
    bool send_mit(PidOutputType output_type) const;
    bool send_state(uint8_t command) const;
};
