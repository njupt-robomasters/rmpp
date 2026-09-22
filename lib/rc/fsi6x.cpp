#include "fsi6x.hpp"
#include "math/math.hpp"
#include <cmath>
#include <cstring> // memcpy

FSi6X::FSi6X(const Config& config) : config(config) {
    // 注册串口回调
    config.uart.add_rx_callback([this](Bytes bytes) {
        this->callback(bytes.data, bytes.size);
    });
}

void FSi6X::on_loop() {
    if (timer_connect_.get_dt() > config.timeout) {
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
    std::memcpy(&raw_, data, sizeof(raw_));

    // 断联标志位检查
    if (raw_.is_disconnect) {
        reset_data();
        return;
    }

    // 复位断联检测
    is_connect = true;
    timer_connect_.update_dt();

    // 解析摇杆和拨杆
    y = -get_joystick(raw_.ch0);    // 右手水平
    x = get_joystick(raw_.ch1);     // 右手垂直
    pitch = get_joystick(raw_.ch2); // 左手垂直
    yaw = -get_joystick(raw_.ch3);  // 左手水平
    swa = get_switch(raw_.ch4);
    swb = get_switch(raw_.ch5);
    swc = get_switch(raw_.ch6);
    swd = get_switch(raw_.ch7);
}

float FSi6X::get_joystick(const uint16_t value) const {
    float ret = ((float)value - 1024.0f) / 783.0f; // 240(-784) ~ 1024 ~ 1807(+783)
    ret = clamp(ret, -1.0f, 1.0f);
    if (std::fabsf(ret) < config.joystick_deadline)
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

void FSi6X::reset_data() {
    is_connect = false;
    x = y = pitch = yaw = 0.0f;
    vra = vrb = 0.0f;
    swb = swc = ERR;
}
