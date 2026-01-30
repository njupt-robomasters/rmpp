#pragma once

#include "Motor.hpp"

class GM6020 : public Motor {
public:
    static constexpr float reduction = 1.0f;
    static constexpr UnitFloat<> Kt = 0.741f * Nm_A;
    static constexpr UnitFloat<> R = 1.8f / 2 * Ohm;

    GM6020(const config_t& config);

    int16_t GetCanCmd() const;

private:
    static constexpr UnitFloat MAX_CURRENT = 3.0f * A;
    static constexpr uint16_t MAX_VOLTAGE_CMD = 25000;
    static constexpr uint16_t MAX_CURRENT_CMD = 16384;

    Angle<> last_angle;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};