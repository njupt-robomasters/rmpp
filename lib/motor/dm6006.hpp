#pragma once

#include "dm_motor.hpp"

class DM6006 : public DmMotor<3.14159f, 45.0f, 12.0f> {
  public:
    static constexpr float MAX_CURRENT = 13.0f;
    static constexpr float MAX_TORQUE = 11.0f;
    static constexpr float REDUCTION = 1.0f;

  public:
    explicit DM6006(const Config& config)
        : DmMotor(config, MAX_CURRENT, MAX_TORQUE, REDUCTION) {}
};
