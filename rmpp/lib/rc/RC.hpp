#pragma once

#include "bsp/bsp.hpp" // dwt

class RC {
public:
    struct config_t {
        UnitFloat<> connect_timeout = 100 * ms;  // 断联检测超时时间
        UnitFloat<> joystick_deadline = 5 * pct; // 摇杆死区，小于此值认为是0
    };

    bool is_connect = false; // 连接标志
    bool is_protect = true;  // 保护标志

    UnitFloat<ratio> x, y, pitch, yaw; // 摇杆

    RC(const config_t& config);

    virtual void OnLoop() = 0;

protected:
    config_t config;

    BSP::Dwt dwt_connect; // 用于断联检测
};