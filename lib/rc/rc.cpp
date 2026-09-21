#include "rc.hpp"

Rc::Rc(const FSi6X::Config& fsi6x_config, const VT13::Config& vt13_config) : vt13_(vt13_config), fsi6x_(fsi6x_config) {}

void Rc::on_loop() {
    vt13_.on_loop();
    fsi6x_.on_loop();

    // 先复位，便于后面叠加控制
    is_enable = false;
    is_rub = is_shoot = is_auto_aim = is_calib = false;
    x = y = r = yaw = pitch = shoot = 0.0f;

    // VT13遥控器
    if (vt13_.is_connect) {
        is_enable |= vt13_.mode == VT13::N || vt13_.mode == VT13::S;

        is_rub |= vt13_.mode == VT13::S;
        is_shoot |= vt13_.trigger;
        is_auto_aim |= vt13_.photo;
        if (vt13_.mode == VT13::C && vt13_.fn && vt13_.photo) {
            is_calib |= true;
        }

        x += vt13_.x, y += vt13_.y, r += vt13_.wheel;
        yaw += vt13_.yaw;

        // pitch摇杆复用
        if (is_shoot) { // 发弹模式，控制弹频
            shoot += vt13_.pitch;
        } else { // 正常pitch
            pitch += vt13_.pitch;
        }

        // 键鼠控制
        mouse = vt13_.mouse;
        key = vt13_.key;
    } else {
        mouse = {};
        key = {};
    }

    // FSi6X遥控器
    if (fsi6x_.is_connect) {
        is_enable |= fsi6x_.swc == FSi6X::MID || fsi6x_.swc == FSi6X::DOWN;

        is_rub |= fsi6x_.swc == FSi6X::DOWN;
        is_shoot |= fsi6x_.swb == FSi6X::DOWN;
        is_auto_aim |= fsi6x_.swd == FSi6X::DOWN;
        if (fsi6x_.swc == FSi6X::UP && fsi6x_.swa == FSi6X::DOWN && fsi6x_.swd == FSi6X::DOWN) {
            is_calib |= true;
        }

        x += fsi6x_.x, y += fsi6x_.y;
        yaw += fsi6x_.yaw;

        // pitch摇杆复用
        if (is_shoot) { // 发弹模式，控制弹频
            shoot += fsi6x_.pitch;
        } else { // 正常pitch
            pitch += fsi6x_.pitch;
        }
    }

    // fn特殊处理
    if (fsi6x_.is_connect) {
        is_fn = fsi6x_.swa == FSi6X::DOWN;
    } else if (vt13_.is_connect) {
        if (vt13_.mode == VT13::N || vt13_.mode == VT13::S) {
            if (vt13_.fn)
                is_fn = true;
            if (vt13_.pause)
                is_fn = false;
        } else {
            is_fn = false;
        }
    } else {
        is_fn = false;
    }
}