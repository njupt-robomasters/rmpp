#pragma once

#include "dm_motor.hpp"

class DM6006 : public DmMotor {
  public:
    struct Config {
        Motor::Config motor;
        float enable_frequency = 10; // 使能帧发送频率，Hz
    };

    static constexpr Motor::Config::Drive DRIVE{.kt = 1, .max_torque = 12, .reduction = 1};
    static constexpr Protocol PROTOCOL{.max_speed = 45, .max_position = 3.14159f};

    explicit DM6006(const Config& config);
};
