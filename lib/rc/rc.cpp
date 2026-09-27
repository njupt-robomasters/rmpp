#include "rc.hpp"
#include "math/math.hpp"
#include <algorithm>

Rc::Rc(const Config& config, FSi6X& fsi6x, VT13& vt13)
    : config(config), fsi6x_(&fsi6x), vt13_(&vt13) {}

Rc::Rc(const Config& config, FSi6X& fsi6x)
    : config(config), fsi6x_(&fsi6x) {}

Rc::Rc(const Config& config, VT13& vt13)
    : config(config), vt13_(&vt13) {}

void Rc::set_ignore_disconnect(bool ignore) {
    ignore_disconnect_ = ignore;
}

void Rc::update() {
    const float dt = timer_update_.restart();
    if (vt13_)
        vt13_->update();
    if (fsi6x_)
        fsi6x_->update();

    const bool vt13_connected = vt13_ && vt13_->is_connect;
    const bool fsi6x_connected = fsi6x_ && fsi6x_->is_connect;

    // 先复位，便于后面叠加控制
    is_enable = ignore_disconnect_;
    is_rub = is_auto_aim = is_calib = false;
    radio_shoot_ = radio_auto_aim_ = false;
    radio_rate_ = 0;
    float x = 0, y = 0, r = 0, yaw = 0, pitch = 0;
    bool calib = false;

    // VT13遥控器
    if (vt13_connected) {
        is_enable |= vt13_->mode == VT13::N || vt13_->mode == VT13::S;

        is_rub |= vt13_->mode == VT13::S;
        radio_shoot_ |= vt13_->trigger;
        radio_auto_aim_ |= vt13_->photo;
        calib |= vt13_->mode == VT13::C && vt13_->fn && vt13_->photo;

        x += vt13_->x;
        y += vt13_->y;
        r += vt13_->wheel;
        yaw += vt13_->yaw;

        // 各遥控器只按自身射击开关决定 pitch 用途
        if (vt13_->trigger) {
            radio_rate_ += vt13_->pitch;
        } else {
            pitch += vt13_->pitch;
        }
    }

    // FSi6X遥控器
    if (fsi6x_connected) {
        is_enable |= fsi6x_->swc == FSi6X::MID || fsi6x_->swc == FSi6X::DOWN;

        is_rub |= fsi6x_->swc == FSi6X::DOWN;
        radio_shoot_ |= fsi6x_->swb == FSi6X::DOWN;
        radio_auto_aim_ |= fsi6x_->swd == FSi6X::DOWN;
        calib |= fsi6x_->swc == FSi6X::UP && fsi6x_->swa == FSi6X::DOWN && fsi6x_->swd == FSi6X::DOWN;

        x += fsi6x_->x;
        y += fsi6x_->y;
        yaw += fsi6x_->yaw;

        if (fsi6x_->swb == FSi6X::DOWN) {
            radio_rate_ += fsi6x_->pitch;
        } else {
            pitch += fsi6x_->pitch;
        }
    }

    // fn特殊处理
    if (fsi6x_connected) {
        is_fn = fsi6x_->swa == FSi6X::DOWN;
    } else if (vt13_connected) {
        if (vt13_->mode == VT13::N || vt13_->mode == VT13::S) {
            if (vt13_->fn)
                is_fn = true;
            if (vt13_->pause)
                is_fn = false;
        } else {
            is_fn = false;
        }
    } else {
        is_fn = false;
    }

    is_calib = calib && !last_calib_;
    last_calib_ = calib;
    update_client(dt);
    vx = x * config.vxy_max + keyboard_vx_;
    vy = y * config.vxy_max + keyboard_vy_;
    wr = r * config.wr_max + keyboard_wr_;
    wyaw = yaw * config.wyaw + mouse_wyaw_;
    wpitch = pitch * config.wpitch + mouse_wpitch_;
    is_auto_aim = radio_auto_aim_ || mouse_right_;
}

static float keyboard_speed(float speed, bool positive, bool negative, float maximum, float acceleration, float deceleration, float dt) {
    if (positive)
        return clamp(std::max(speed, 0.0f) + acceleration * dt, maximum);
    if (negative)
        return clamp(std::min(speed, 0.0f) - acceleration * dt, maximum);
    if (speed > 0)
        return std::max(0.0f, speed - deceleration * dt);
    return std::min(0.0f, speed + deceleration * dt);
}

void Rc::update_client(float dt) {
    is_turn_back = is_refresh_ui = false;
    bullet_step = 0;
    if (!vt13_ || !vt13_->is_connect) {
        keyboard_vx_ = keyboard_vy_ = keyboard_wr_ = 0;
        mouse_wyaw_ = mouse_wpitch_ = 0;
        mouse_left_ = mouse_right_ = force_fire_ = has_click_ = false;
        last_r_ = last_z_ = last_c_ = false;
        return;
    }

    const auto key = vt13_->key;
    const auto mouse = vt13_->mouse;
    keyboard_vx_ = keyboard_speed(keyboard_vx_, key.w, key.s, config.vxy_max, config.axy, config.dxy, dt);
    keyboard_vy_ = keyboard_speed(keyboard_vy_, key.a, key.d, config.vxy_max, config.axy, config.dxy, dt);
    keyboard_wr_ = keyboard_speed(keyboard_wr_, key.shift, key.ctrl, config.wr_max, config.ar, config.dr, dt);
    mouse_wyaw_ += clamp(mouse.yaw * config.wyaw - mouse_wyaw_, config.ayaw * dt);
    mouse_wpitch_ += clamp(mouse.pitch * config.wpitch - mouse_wpitch_, config.apitch * dt);

    is_turn_back = key.r && !last_r_;
    is_refresh_ui = key.f;
    bullet_step = static_cast<int8_t>(key.c && !last_c_) - static_cast<int8_t>(key.z && !last_z_);
    last_r_ = key.r;
    last_z_ = key.z;
    last_c_ = key.c;

    // 每周期让点击记录过期，避免长按右键跨过计数器回绕后误判双击
    if (has_click_ && timer_click_.elapsed() >= 250 * ms) {
        timer_click_.restart();
        has_click_ = false;
    }
    if (!mouse.right) {
        force_fire_ = has_click_ = false;
    } else {
        if (mouse.left && !mouse_left_) {
            force_fire_ = has_click_ && timer_click_.elapsed() < 250 * ms;
            timer_click_.restart();
            has_click_ = true;
        }
        if (!mouse.left)
            force_fire_ = false;
    }
    mouse_left_ = mouse.left;
    mouse_right_ = mouse.right;
}

Rc::ShootCommand Rc::shoot_command(bool vision_fire) const {
    if (radio_shoot_ && (!radio_auto_aim_ || vision_fire))
        return {.is_shoot = true, .rate = radio_rate_};
    if (mouse_left_ && (!mouse_right_ || force_fire_ || vision_fire))
        return {.is_shoot = true, .rate = 1};
    return {};
}
