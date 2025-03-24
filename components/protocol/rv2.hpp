#pragma once

#include <cstdint>

class RV2 {
public:
    float pitch, yaw;
    bool is_locked, fire_advise;

    void SendIMUData(float roll, float pitch, float yaw);

    void SendRefereeData(bool team_is_red, float bullet_speed);

    void ParseStreamingData(const uint8_t *data, uint32_t len);
};
