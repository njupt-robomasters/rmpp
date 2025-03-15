#pragma once

#include <cstdint>

class RV2 {
public:
    float pitch, yaw;
    bool fire_advise;

    void SendIMUData(float roll, float pitch, float yaw);

    void ParseStreamingData(const uint8_t *data, int len);
};
