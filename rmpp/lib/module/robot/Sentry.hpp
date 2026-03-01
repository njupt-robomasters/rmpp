#pragma once

#include "Infantry.hpp"

class Sentry : public Infantry {
public:
    using Infantry::Infantry;

    void OnLoop();

private:


    //底盘扩展mavlink速度控制
    struct Nav_Vel{
        UnitFloat<m_s> mavlink;
    }nav_vx, nav_vy;

    enum {
        ORIGIN,
        POS1,
        POS2
    } nav_status = ORIGIN;

    void handleDebug();

    void handleChassis() override;
};