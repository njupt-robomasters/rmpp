#pragma once

#include "dm_motor.hpp"

class DM4310 : public DmMotor {
  public:
    struct Config {
        Motor::Config motor;
        float enable_frequency = 10; // 使能帧发送频率，Hz
    };

    static constexpr Motor::Config::Drive DRIVE{.kt = 1.2f, .max_torque = 10, .reduction = 1};
    static constexpr Protocol PROTOCOL{.max_speed = 30, .max_position = 3.14159f};

    explicit DM4310(const Config& config);
};
