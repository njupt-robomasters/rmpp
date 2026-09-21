#pragma once

#include "Motor.hpp"

class M2006 : public Motor {
public:
    static constexpr float REDUCTION = 36.0f;
    static constexpr float Kt = 0.18f;
    static constexpr float R = 0.461f / 2;
    static constexpr float MAX_CURRENT = 10.0f;
    static constexpr float MAX_TORQUE = MAX_CURRENT * Kt;

    M2006(const config_t& config);

    int16_t GetCanCmd() const;

private:
    static constexpr uint16_t MAX_CURRENT_CMD = 10000;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};