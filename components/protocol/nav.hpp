#pragma once

#include <cstdint>

class NAV {
public:
    float position_x = 0, position_y = 0, position_yaw = 0; //当前车的位置
    float vel_x = 0, vel_y = 0, vel_yaw = 0; //目标速度

    void SendTargetData(float x, float y);

    void ParseStreamingData(const uint8_t *data, uint32_t len);
};
