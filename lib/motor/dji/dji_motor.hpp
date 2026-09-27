#pragma once

#include "motor/motor.hpp"

class DjiMotor : public Motor {
  public:
    struct Protocol {
        int16_t feedback_scale = 0, command_scale = 0; // 电流反馈、命令满量程刻度
    } const protocol;

    int16_t command_value() const;

  protected:
    DjiMotor(const Config& config, Protocol protocol);
    void receive(const CanFrame& frame) final;
    // 默认布局为编码器、速度、电流，特殊型号只重写解析
    virtual void parse_feedback(const uint8_t data[8]);
    void accept_dji_feedback(uint16_t encoder, int16_t current, int16_t speed = 0);
};
