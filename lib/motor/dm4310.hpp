#pragma once

#include "dm_motor.hpp"

class DM4310 : public DmMotor<3.14159f, 30.0f, 10.0f> {
  public:
    static constexpr float MAX_CURRENT = 7.5f;
    static constexpr float MAX_TORQUE = 7.0f;
    static constexpr float KT = 2.5f / 3.0f;
    static constexpr float REDUCTION = 1.0f;

    explicit DM4310(const Config& config)
        : DmMotor(config) {}
};
