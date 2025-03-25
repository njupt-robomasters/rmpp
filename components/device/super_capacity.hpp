#pragma once

#include <cstdint>

class SuperCapacity {
public:
    float input_voltage = 0;
    float capacity_voltage = 0;
    float input_current = 0;
    float power_limit = 0;

    float input_power = 0;
    float percentage = 0;

    void ParseCAN(uint32_t id, const uint8_t data[8]);

    void SendPowerLimit(float power);

    bool isLowEnergy() const;
};
