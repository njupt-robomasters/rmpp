#pragma once

#include "motor.hpp"

class MF9025 : public Motor {
  public:
    static constexpr float REDUCTION = 1.0f;
    static constexpr float KT = 0.32f;
    static constexpr float MAX_CURRENT = 16.5f;
    static constexpr float MAX_TORQUE = MAX_CURRENT * KT;

    MF9025(const Config& config);

    void send_can_cmd();

  private:
    static constexpr uint16_t MAX_CURRENT_CMD = 2048;

    uint32_t send_cnt_ = 0; // 用于定期发送使能命令

    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    // 发送电流
    void send_current(float current) const;
};