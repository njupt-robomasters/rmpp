#include "RC.hpp"

RC::RC(const FSi6X::config_t& fsi6x_config, const VT13::config_t& vt13_config) : fsi6x(fsi6x_config), vt13(vt13_config) {}

void RC::OnLoop() {
    fsi6x.OnLoop();
    vt13.OnLoop();

    if (vt13.is_connect) { // VT13遥控器优先
        is_enable = vt13.mode == VT13::N || vt13.mode == VT13::S;

        x = vt13.x;
        y = vt13.y;
        r = vt13.wheel;

        yaw = vt13.yaw;
        pitch = vt13.pitch;

        is_rub = vt13.mode == VT13::S;
        is_shoot = vt13.trigger;
    } else if (fsi6x.is_connect) {
        is_enable = fsi6x.swa == FSi6X::DOWN;

        x = fsi6x.x;
        y = fsi6x.y;
        if (fsi6x.swb == FSi6X::DOWN) {
            r = fsi6x.vra;
        } else {
            r = 0 * ratio;
        }

        yaw = fsi6x.yaw;
        pitch = fsi6x.pitch;

        is_rub = fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN;
        is_shoot = fsi6x.swc == FSi6X::DOWN;
    } else {
        is_enable = false;
        x = y = r = yaw = pitch = 0 * ratio;
        is_rub = is_shoot = false;
    }
}