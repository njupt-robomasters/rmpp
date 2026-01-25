#pragma once

#include "unit/include_me.hpp"
#include "bsp/bsp.hpp"

class UI {
public:
    // yaw
    Angle<deg> yaw;

    // hit
    bool is_hit = false;
    Angle<deg> hit;

    // n630_speed
    UnitFloat<m_s> bullet_speed_1;
    UnitFloat<m_s> bullet_speed_2;

    // shoot_current
    UnitFloat<A> shoot_current;

    // enemy_hp
    uint16_t enemy_hp = 200;

    // aim
    bool is_aim_connected = false, is_aim_detected = false;

    void Init();

    void OnLoop();

private:
    static constexpr UnitFloat<> UPDATE_FREQ = 10.0f * Hz;

    uint8_t state = 0;

    BSP::Dwt dwt_update_freq;

    void updateLib();
};