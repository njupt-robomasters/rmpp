#pragma once

#include "Motor.hpp"

class M3508 : public Motor {
public:
    static constexpr float reduction = 3591.0f / 187.0f;
    static constexpr UnitFloat<> Kt = 0.3f * Nm_A;
    static constexpr UnitFloat<> R = 0.194f / 2 * Ohm;

    M3508(const config_t& config);

    int16_t GetCanCmd() const;

    void SendQuickSetID() const;

private:
    static constexpr UnitFloat MAX_CURRENT = 20.0f * A;
    static constexpr uint16_t MAX_CURRENT_CMD = 16384;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};