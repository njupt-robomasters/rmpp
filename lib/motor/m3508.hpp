#pragma once

#include "motor.hpp"

class M3508 : public Motor {
  public:
    static constexpr float REDUCTION = 3591.0f / 187.0f;
    static constexpr float KT = 0.3f;
    static constexpr float R = 0.194f / 2;
    static constexpr float MAX_CURRENT = 20.0f;
    static constexpr float MAX_TORQUE = MAX_CURRENT * KT;

    M3508(const Config& config);

    int16_t get_can_cmd() const;

    void send_quick_set_id() const;

  private:
    static constexpr uint16_t MAX_CURRENT_CMD = 16384;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};