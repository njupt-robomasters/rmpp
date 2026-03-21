#include "RC.hpp"

RC::RC(const FSi6X::config_t& fsi6x_config, const VT13::config_t& vt13_config) : vt13(vt13_config), fsi6x(fsi6x_config) {}

void RC::OnLoop() {
    vt13.OnLoop();
    fsi6x.OnLoop();

    // 先复位，便于后面叠加控制
    is_enable = false;
    is_rub = is_shoot = is_auto_aim = false;
    x = y = r = yaw = pitch = shoot = 0 * ratio;

    // VT13遥控器
    if (vt13.is_connect) {
        is_enable |= vt13.mode == VT13::N || vt13.mode == VT13::S;

        is_rub |= vt13.mode == VT13::S;
        is_shoot |= vt13.trigger;
        is_auto_aim |= vt13.photo;

        x += vt13.x, y += vt13.y, r += vt13.wheel;
        yaw += vt13.yaw;

        // pitch摇杆复用
        if (is_shoot) { // 发弹模式，控制弹频
            shoot += vt13.pitch;
        } else { // 正常pitch作用
            pitch += vt13.pitch;
        }

        // 键鼠控制
        mouse = vt13.mouse;
        key = vt13.key;
    } else {
        mouse = {};
        key = {};
    }

    // FSi6X遥控器
    if (fsi6x.is_connect) {
        is_enable |= true;

        is_rub |= fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN;
        is_shoot |= fsi6x.swc == FSi6X::DOWN;
        is_auto_aim |= fsi6x.swd == FSi6X::DOWN;

        x += fsi6x.x, y += fsi6x.y;
        yaw += fsi6x.yaw;

        // pitch摇杆复用
        if (is_shoot) { // 发弹模式，控制弹频
            shoot += fsi6x.pitch;
        } else if (fsi6x.swb == FSi6X::DOWN) { // 小陀螺模式，控制转速
            r += fsi6x.pitch;
        } else { // 正常pitch作用
            pitch += fsi6x.pitch;
        }
    }

    // fn特殊处理
    if (fsi6x.is_connect) {
        is_fn = fsi6x.swa == FSi6X::DOWN;
    } else if (vt13.is_connect) {
        if (vt13.fn) is_fn = true;
        if (vt13.pause) is_fn = false;
    } else {
        is_fn = false;
    }
}