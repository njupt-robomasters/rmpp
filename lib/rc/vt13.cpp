#include "vt13.hpp"
#include "referee/crc.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

VT13::VT13(const Config& config)
    : config(config) {
    // 注册串口回调
    config.uart.add_rx_callback([this](const uint8_t* data, std::size_t size) {
        callback(data, size);
    });
}

void VT13::update() {
    // 断联检测
    if (callback_timer.elapsed() > config.timeout) {
        callback_timer.restart();
        reset_data();
    }
}

void VT13::callback(const uint8_t data[], const size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (rx_size == sizeof(rx_buf))
            rx_size = 0;
        rx_buf[rx_size++] = data[i];

        while (rx_size >= 2) {
            if (rx_buf[0] != 0xA9 || rx_buf[1] != 0x53) {
                std::memmove(rx_buf, rx_buf + 1, --rx_size);
                continue;
            }
            if (rx_size < sizeof(raw))
                break;

            const uint16_t crc16 = rx_buf[19] | (static_cast<uint16_t>(rx_buf[20]) << 8);
            if (!CRC16::verify(rx_buf, 19, crc16)) {
                std::memmove(rx_buf, rx_buf + 1, --rx_size);
                continue;
            }

            is_connect = true;
            callback_timer.restart();
            std::memcpy(&raw, rx_buf, sizeof(raw));
            std::memmove(rx_buf, rx_buf + sizeof(raw), rx_size - sizeof(raw));
            rx_size -= sizeof(raw);
            parse_raw();
        }
    }
}

void VT13::parse_raw() {

    // 解析摇杆
    y = -get_joy_stick(raw.ch_0);      // 右手水平
    x = get_joy_stick(raw.ch_1);       // 右手垂直
    pitch = get_joy_stick(raw.ch_2);   // 左手垂直
    yaw = -get_joy_stick(raw.ch_3);    // 左手水平
    wheel = -get_joy_stick(raw.wheel); // 拨轮

    // 解析开关和按钮
    mode = Mode(raw.mode_sw);
    pause = raw.pause;
    trigger = raw.trigger;
    fn = raw.fn_1;
    photo = raw.fn_2;

    // 解析鼠标
    mouse.yaw = -static_cast<float>(raw.mouse_x) / static_cast<float>(config.mouse_max);
    mouse.pitch = static_cast<float>(raw.mouse_y) / static_cast<float>(config.mouse_max);
    mouse.wheel = raw.mouse_z;
    mouse.left = raw.mouse_left;
    mouse.right = raw.mouse_right;
    mouse.middle = raw.mouse_middle;

    // 解析键盘
    key.w = raw.key & (1 << 0);
    key.s = raw.key & (1 << 1);
    key.a = raw.key & (1 << 2);
    key.d = raw.key & (1 << 3);
    key.shift = raw.key & (1 << 4);
    key.ctrl = raw.key & (1 << 5);
    key.q = raw.key & (1 << 6);
    key.e = raw.key & (1 << 7);
    key.r = raw.key & (1 << 8);
    key.f = raw.key & (1 << 9);
    key.g = raw.key & (1 << 10);
    key.z = raw.key & (1 << 11);
    key.x = raw.key & (1 << 12);
    key.c = raw.key & (1 << 13);
    key.v = raw.key & (1 << 14);
    key.b = raw.key & (1 << 15);
}

void VT13::reset_data() {
    is_connect = false;

    // 遥控器
    x = y = pitch = yaw = 0.0f;
    wheel = 0.0f;
    mode = ERR;
    pause = trigger = fn = photo = false;

    // 键鼠
    mouse = {};
    key = {};
}

float VT13::get_joy_stick(const uint16_t value) const {
    float ret = (static_cast<float>(value) - 1024.0f) / 660.0f;
    ret = std::clamp(ret, -1.0f, 1.0f);
    if (std::fabs(ret) < config.deadline)
        ret = 0.0f;
    return ret;
}
