#pragma once

#include "Motor.hpp"

class RM6623 : public Motor {
public:
    static constexpr float reduction = 1.0f;
    static constexpr UnitFloat<> Kt = 0.38f * Nm_A;

    RM6623(const config_t& config);

    int16_t GetCurrentCmd() const;

    // 发送电机校准指令（更换电机/电调时使用）
    void SendCalibrateCmd() const;

private:
    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};