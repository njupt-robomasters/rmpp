#include <cstdint>
#pragma once

class NAV {
public:
    void SendTargetData( float x,  float y,  float yaw);
    void SendRefereeData(bool team_is_red, float bullet_speed);
    void ParseStreamingData(const uint8_t *data, uint32_t len);

private:
};

