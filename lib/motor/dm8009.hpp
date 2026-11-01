#pragma once

#include "dm_motor.hpp"

class DM8009 : public DmMotor<3.14159f, 45.0f, 54.0f> {
  public:
    static constexpr float MAX_CURRENT = 40.0f;
    static constexpr float MAX_TORQUE = 40.0f;
    static constexpr float KT = 1.0f;
    static constexpr float REDUCTION = 1.0f;

    explicit DM8009(const Config& config)
        : DmMotor(config) {}
};
