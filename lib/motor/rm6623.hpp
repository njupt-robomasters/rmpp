#pragma once

#include "motor.hpp"

class RM6623 : public Motor {
  public:
    static constexpr float REDUCTION = 1.0f;
    static constexpr float KT = 0.38f;
    static constexpr float MAX_CURRENT = 5.0f;
    static constexpr float MAX_TORQUE = MAX_CURRENT * KT;

    RM6623(const Config& config);

    int16_t get_can_cmd() const;

    // 发送电机校准指令（更换电机/电调时使用）
    void send_calibrate_cmd() const;

  private:
    static constexpr uint16_t MAX_CURRENT_CMD = 5000;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};