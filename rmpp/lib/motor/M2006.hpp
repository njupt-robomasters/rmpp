#pragma once

#include "Motor.hpp"

class M2006 : public Motor {
public:
    static constexpr float reduction = 36.0f;
    static constexpr UnitFloat<> Kt = 0.18f * Nm_A;
    static constexpr UnitFloat<> R = 0.461f / 2 * Ohm;

    M2006(const config_t& config);

    int16_t GetCurrentCmd() const;

private:
    static constexpr UnitFloat MAX_CURRENT = 10.0f * A;
    static constexpr uint16_t MAX_CURRENT_CMD = 10000;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};