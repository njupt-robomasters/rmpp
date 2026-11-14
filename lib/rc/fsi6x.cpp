#include "fsi6x.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

FSi6X::FSi6X(const Config& config)
    : timeout(config.timeout),
      deadline(config.deadline) {
    // 注册串口回调
    config.uart.add_rx_callback([this](const uint8_t* data, const size_t size) {
        callback(data, size);
    });
}

void FSi6X::update() {
    if (callback_timer.elapsed() > timeout) {
        callback_timer.restart();
        reset_data();
    }
}

void FSi6X::callback(const uint8_t data[], const uint16_t size) {
    // 长度检查
    if (size != 25)
        return;

    // 帧头桢尾检查
    if (data[0] != 0x0F || data[24] != 0x00)
        return;

    // 解析原始通道值
    std::memcpy(&raw, data, sizeof(raw));

    // 断联标志位检查
    if (raw.is_disconnect) {
        reset_data();
    } else {
        // 复位断联检测
        is_connect = true;
        callback_timer.restart();

        // 解析摇杆和拨杆
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
}

void FSi6X::reset_data() {
    is_connect = false;
    x = y = pitch = yaw = 0.0f;
    swa = swb = swc = swd = ERR;
    vra = vrb = 0.0f;
}

float FSi6X::get_joystick(const uint16_t value) const {
    float ret = (static_cast<float>(value) - 1024.0f) / 783.0f; // 240(-784) ~ 1024 ~ 1807(+783)
    ret = std::clamp(ret, -1.0f, 1.0f);
    if (std::fabs(ret) < deadline)
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
