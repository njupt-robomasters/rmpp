#include "fsi6x.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

FSi6X::FSi6X(const Config& config)
    : config(config) {
    config.uart.add_rx_callback([this](const uint8_t* data, const size_t size) { callback(data, size); });
}

void FSi6X::update() {
    // 断联检测：操控量复位，原始帧保留
    if (callback_timer.elapsed() > config.timeout)
        reset();
}

void FSi6X::callback(const uint8_t* data, const size_t size) {
    // SBUS 帧：空闲中断一次收齐 25 B，0x0F 帧头、0x00 帧尾，无校验
    if (size != sizeof(raw) || data[0] != 0x0Fu || data[sizeof(raw) - 1] != 0x00u)
        return;
    std::memcpy(&raw, data, sizeof(raw));

    // 接收机失控：立即复位，不刷新连接
    if (raw.is_disconnect) {
        reset();
        return;
    }

    // 有效帧：解析并刷新连接
    parse_raw();
    is_connect = true;
    callback_timer.restart();
}

void FSi6X::parse_raw() {
    // 解析摇杆
    y = -get_joystick(raw.ch0);    // 右手水平
    x = get_joystick(raw.ch1);     // 右手垂直
    pitch = get_joystick(raw.ch2); // 左手垂直
    yaw = -get_joystick(raw.ch3);  // 左手水平

    // 解析拨杆
    swa = get_switch(raw.ch4);
    swb = get_switch(raw.ch5);
    swc = get_switch(raw.ch6);
    swd = get_switch(raw.ch7);

    // 解析旋钮
    vra = get_joystick(raw.ch8);
    vrb = get_joystick(raw.ch9);
}

float FSi6X::get_joystick(const uint16_t value) const {
    float ret = (static_cast<float>(value) - 1024.0f) / 783.0f;
    ret = std::clamp(ret, -1.0f, 1.0f);
    if (std::fabs(ret) < config.dead_zone)
        ret = 0.0f;
    return ret;
}

FSi6X::Switch FSi6X::get_switch(const uint16_t value) {
    if (value == 240)
        return UP;
    if (value == 1024)
        return MID;
    if (value == 1807)
        return DOWN;
    return ERR;
}

void FSi6X::reset() {
    is_connect = false;
    x = y = pitch = yaw = 0;
    swa = swb = swc = swd = ERR;
    vra = vrb = 0;
}
