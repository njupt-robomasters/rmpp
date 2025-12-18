#pragma once

#include "lib/ui.h"
#include "unit/include_me.hpp"
#include "bsp/bsp.hpp"

class UI {
public:
    bool is_detected = false, is_locked = false;
    Angle<deg> yaw;
    UnitFloat<pct> cap;
    Angle<deg> hit;

    void Init();

    void OnLoop();

private:
    static constexpr UnitFloat UPDATE_FREQ = 10.0f * Hz;

    uint8_t state = 0;

    BSP::Dwt dwt_update_freq;

    void updateLib();
};