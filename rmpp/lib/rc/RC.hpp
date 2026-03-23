#pragma once

#include "FSi6X.hpp"
#include "VT13.hpp"

class RC {
public:
    bool is_enable = false;

    bool is_rub = false, is_shoot = false, is_auto_aim = false, is_fn = false, is_calib = false;
    UnitFloat<ratio> x, y, r, pitch, yaw, shoot;

    VT13::mouse_t mouse;
    VT13::key_t key;

    RC(const FSi6X::config_t& fsi6x_config, const VT13::config_t& vt13_config);

    // 需要在循环中调用
    void OnLoop();

private:
    VT13 vt13;
    FSi6X fsi6x;
};