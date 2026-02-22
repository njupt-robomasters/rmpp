#pragma once

#include "FSi6X.hpp"
#include "VT13.hpp"

class RC {
public:
    bool is_enable = false;

    UnitFloat<ratio> x, y, r, pitch, yaw;
    bool is_rub = false, is_shoot = false;

    FSi6X fsi6x;
    VT13 vt13;

    RC(const FSi6X::config_t& fsi6x_config, const VT13::config_t& vt13_config);

    // 需要在循环中调用
    void OnLoop();
};