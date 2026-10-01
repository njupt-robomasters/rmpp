#pragma once

#include "motor/motor.hpp"

class DjiMotor : public Motor {
  public:
    // 将本电机命令写入组帧对应位置，由 app 统一发送
    virtual bool write_can_frame(CanFrame& frame) const = 0;

  protected:
    explicit DjiMotor(const Config& config);

    float directed_current() const;
    bool write_command(CanFrame& frame, int16_t command, uint32_t first_rx_id, uint8_t motor_count = 4) const;
    void can_callback(uint16_t encoder, float current, int16_t speed);
};
