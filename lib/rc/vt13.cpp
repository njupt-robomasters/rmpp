#include "vt13.hpp"
#include "referee/crc.hpp"
#include <algorithm>
#include <cmath>

VT13::VT13(const Config& config)
    : config(config) {
    config.uart.add_rx_callback([this](const uint8_t* data, const size_t size) { callback(data, size); });
}

void VT13::add_0311_callback(RxCallback callback) {
    callbacks_0311.push_back(std::move(callback));
}

void VT13::update() {
    const float dt = std::clamp(update_timer.restart(), 0.0001f, 0.01f);

    // 断联检测：原始量与键盘斜坡复位，操控量随之为 0
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        rc = {};
        mouse = {};
        key = {};
        key_vx = key_vy = key_wr = 0;
    }

    // 挡位与自瞄；断联时挡位为 ERR
    is_enable = rc.mode == N || rc.mode == S;
    bullet_speed = rc.mode == S ? config.bullet_speed : 0;
    is_aim = rc.photo || mouse.right;

    // 底盘：摇杆叠加键盘斜坡，W/S 前后、A/D 左右、Shift/Ctrl 逆/顺时针
    key_vx = get_key_speed(key_vx, key.w, key.s, config.vxy_max, config.axy, config.dxy, dt);
    key_vy = get_key_speed(key_vy, key.a, key.d, config.vxy_max, config.axy, config.dxy, dt);
    key_wr = get_key_speed(key_wr, key.shift, key.ctrl, config.wr_max, config.ar, config.dr, dt);
    vx = std::clamp(rc.x * config.vxy_max + key_vx, -config.vxy_max, config.vxy_max);
    vy = std::clamp(rc.y * config.vxy_max + key_vy, -config.vxy_max, config.vxy_max);
    wr = std::clamp(rc.wheel * config.wr_max + key_wr, -config.wr_max, config.wr_max);

    // 云台：摇杆叠加鼠标，摇杆上推抬头；按住 trigger 时 pitch 摇杆改为调弹频
    const float pitch_stick = rc.trigger ? 0 : rc.pitch;
    yaw_speed = std::clamp((rc.yaw + mouse.yaw) * config.yaw_speed_max, -config.yaw_speed_max, config.yaw_speed_max);
    pitch_speed = std::clamp(-(pitch_stick + mouse.pitch) * config.pitch_speed_max, -config.pitch_speed_max, config.pitch_speed_max);

    // 弹频：滚轮由 0 上跳加一步、下跳减一步（只看正负号），限幅到 [min, max]
    if (last_mouse_wheel == 0 && mouse.wheel > 0)
        config.bullet_freq = std::min(config.bullet_freq + config.bullet_freq_step, config.bullet_freq_max);
    else if (last_mouse_wheel == 0 && mouse.wheel < 0)
        config.bullet_freq = std::max(config.bullet_freq - config.bullet_freq_step, config.bullet_freq_min);
    last_mouse_wheel = mouse.wheel;

    // 发射：trigger 时 pitch 摇杆为弹频倍率，负值反转；否则鼠标左键按弹频
    if (rc.trigger)
        bullet_freq = rc.pitch * config.bullet_freq;
    else if (rc.mode == S && mouse.left)
        bullet_freq = config.bullet_freq;
    else
        bullet_freq = 0;
}

void VT13::callback(const uint8_t* data, const size_t size) {
    for (size_t index = 0; index < size; index++) {
        rx_fifo.push(data[index]);
        while (rx_fifo.size >= 2) {
            // 遥控帧：A9 53 帧头，定长
            if (rx_fifo[0] == 0xA9 && rx_fifo[1] == 0x53) {
                if (rx_fifo.size < sizeof(raw))
                    break;

                // CRC 不符：丢弃 1 字节，帧内出现的帧头可重新对齐
                decltype(raw) frame;
                auto* bytes = reinterpret_cast<uint8_t*>(&frame);
                rx_fifo.peek(bytes, sizeof(frame));
                if (crc16_referee(bytes, sizeof(frame) - 2) != frame.crc16) {
                    rx_fifo.pop(1);
                    continue;
                }
                rx_fifo.pop(sizeof(frame));

                // 有效帧：保存原始帧并解析
                raw = frame;
                parse_raw();
                is_connect = true;
                callback_timer.restart();
                continue;
            }

            // 图传链路裁判帧：A5 帧头，帧格式同常规链路；只分发 0x0311
            if (rx_fifo[0] == 0xA5) {
                if (rx_fifo.size < 5)
                    break;

                // 帧头 5 B：A5、载荷长度 2 B、包序号、CRC8；载荷超长或 CRC8 不符时丢弃 1 字节重新对齐
                uint8_t frame[COMMAND_MAX + 9];
                rx_fifo.peek(frame, 5);
                const size_t payload_size = frame[1] | (static_cast<size_t>(frame[2]) << 8u);
                if (payload_size > COMMAND_MAX || crc8_referee(frame, 4) != frame[4]) {
                    rx_fifo.pop(1);
                    continue;
                }
                const size_t frame_size = 5 + 2 + payload_size + 2;
                if (rx_fifo.size < frame_size)
                    break;

                // 整帧：帧头、命令码 2 B、载荷、CRC16 2 B；CRC16 不符时丢弃 1 字节重新对齐
                rx_fifo.peek(frame, frame_size);
                const uint16_t crc = frame[frame_size - 2] | (static_cast<uint16_t>(frame[frame_size - 1]) << 8u);
                if (crc16_referee(frame, frame_size - 2) != crc) {
                    rx_fifo.pop(1);
                    continue;
                }
                rx_fifo.pop(frame_size);

                // 有效帧：载荷仅在回调期间有效，其他命令码直接丢弃
                const uint16_t cmd_id = frame[5] | (static_cast<uint16_t>(frame[6]) << 8u);
                if (cmd_id == 0x0311)
                    for (const auto& rx_callback : callbacks_0311)
                        rx_callback(frame + 7, payload_size);
                continue;
            }

            // 帧头不符：丢弃 1 字节重新对齐
            rx_fifo.pop(1);
        }
    }
}

void VT13::parse_raw() {
    // 解析摇杆
    rc.y = -get_joystick(raw.ch_0);      // 右手水平
    rc.x = get_joystick(raw.ch_1);       // 右手垂直
    rc.pitch = get_joystick(raw.ch_2);   // 左手垂直
    rc.yaw = -get_joystick(raw.ch_3);    // 左手水平
    rc.wheel = -get_joystick(raw.wheel); // 拨轮

    // 解析开关和按钮
    rc.mode = Mode(raw.mode_sw);
    rc.pause = raw.pause;
    rc.trigger = raw.trigger;
    rc.fn = raw.fn_1;
    rc.photo = raw.fn_2;

    // 解析鼠标
    mouse.yaw = -static_cast<float>(raw.mouse_x) / static_cast<float>(config.mouse_max);
    mouse.pitch = static_cast<float>(raw.mouse_y) / static_cast<float>(config.mouse_max);
    mouse.wheel = raw.mouse_z;
    mouse.left = raw.mouse_left;
    mouse.right = raw.mouse_right;
    mouse.middle = raw.mouse_middle;

    // 解析键盘
    key.w = raw.key & (1u << 0u);
    key.s = raw.key & (1u << 1u);
    key.a = raw.key & (1u << 2u);
    key.d = raw.key & (1u << 3u);
    key.shift = raw.key & (1u << 4u);
    key.ctrl = raw.key & (1u << 5u);
    key.q = raw.key & (1u << 6u);
    key.e = raw.key & (1u << 7u);
    key.r = raw.key & (1u << 8u);
    key.f = raw.key & (1u << 9u);
    key.g = raw.key & (1u << 10u);
    key.z = raw.key & (1u << 11u);
    key.x = raw.key & (1u << 12u);
    key.c = raw.key & (1u << 13u);
    key.v = raw.key & (1u << 14u);
    key.b = raw.key & (1u << 15u);
}

float VT13::get_joystick(const uint16_t value) const {
    float ret = (static_cast<float>(value) - 1024.0f) / 660.0f;
    ret = std::clamp(ret, -1.0f, 1.0f);
    if (std::fabs(ret) < config.dead_zone)
        ret = 0.0f;
    return ret;
}

float VT13::get_key_speed(const float speed, const bool positive, const bool negative, const float max, const float acceleration, const float deceleration, const float dt) {
    if (positive)
        return std::clamp(std::max(speed, 0.0f) + acceleration * dt, -max, max);
    if (negative)
        return std::clamp(std::min(speed, 0.0f) - acceleration * dt, -max, max);
    if (speed > 0)
        return std::max(0.0f, speed - deceleration * dt);
    return std::min(0.0f, speed + deceleration * dt);
}
