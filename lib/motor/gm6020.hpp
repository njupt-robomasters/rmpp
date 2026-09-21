#pragma once

#include "math/angle.hpp"
#include "motor.hpp"

using math::Angle;

class GM6020 : public Motor {
  public:
    static constexpr float REDUCTION = 1.0f;
    static constexpr float KT = 0.741f;
    static constexpr float R = 1.8f / 2;
    static constexpr float MAX_CURRENT = 3.0f;
    static constexpr float MAX_TORQUE = MAX_CURRENT * KT;

    GM6020(const Config& config);

    int16_t get_can_cmd() const;

  private:
    static constexpr uint16_t MAX_VOLTAGE_CMD = 25000;
    static constexpr uint16_t MAX_CURRENT_CMD = 16384;

    Angle last_angle_;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};