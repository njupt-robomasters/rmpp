#pragma once

#include "motor.hpp"

class CustomMotor : public Motor {
  public:
    CustomMotor(const Config& config);

    int16_t get_can_cmd() const;

  private:
    // CAN接收回调
    void callback(uint32_t id, const uint8_t data[8], uint8_t dlc);
};