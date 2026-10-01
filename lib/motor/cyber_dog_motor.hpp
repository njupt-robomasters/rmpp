#pragma once

#include "motor.hpp"

class CyberDogMotor : public Motor {
  public:
    using Config = Motor::Config;

    static constexpr Config::Drive DRIVE{.kt = 0.9f, .max_current = 40, .reduction = 1};
    static constexpr float MAX_SPEED = 65;       // rad/s
    static constexpr float MAX_POSITION = 12.5f; // rad

    explicit CyberDogMotor(const Config& config);

    void send();

  private:
    uint32_t send_count_ = 0;

    void can_callback(const CanFrame& frame);
    void send_mit() const;
    void send_enable() const;
    void send_disable() const;
};
