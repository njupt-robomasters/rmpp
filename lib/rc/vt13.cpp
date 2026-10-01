#include "vt13.hpp"
#include "referee/lib/crc.hpp"
#include <algorithm>
#include <cstring>

VT13::VT13(const Config& config)
    : config(config) {
    // 注册串口回调
    config.uart.add_rx_callback([this](const uint8_t* data, std::size_t size) {
        this->callback(data, size);
    });
}

void VT13::update() {
    // 断联检测
    if (timer_connect_.elapsed() > config.timeout) {
        timer_connect_.restart();
        reset_data();
    }
}

void VT13::callback(const uint8_t data[], const uint16_t size) {
    // 长度检查
    if (size != 21)
        return;

    // 帧头检查
    if (data[0] != 0xA9 || data[1] != 0x53)
        return;

    // CRC16校验
    const uint16_t crc16 = data[19] | (data[20] << 8);
    if (!CRC16::verify(data, 19, crc16))
        return;

    // 复位断联检测
    is_connect = true;
    timer_connect_.restart();

    // 解析原始通道值
    std::memcpy(&raw_, data, sizeof(raw_));

    // 解析摇杆
    y = -get_joy_stick(raw_.ch_0);      // 右手水平
    x = get_joy_stick(raw_.ch_1);       // 右手垂直
    pitch = get_joy_stick(raw_.ch_2);   // 左手垂直
    yaw = -get_joy_stick(raw_.ch_3);    // 左手水平
    wheel = -get_joy_stick(raw_.wheel); // 拨轮

    // 解析开关和按钮
    mode = Mode(raw_.mode_sw);
    pause = raw_.pause;
    trigger = raw_.trigger;
    fn = raw_.fn_1;
    photo = raw_.fn_2;

    // 解析鼠标
    mouse.yaw = -static_cast<float>(raw_.mouse_x) / static_cast<float>(config.mouse_max);
    mouse.pitch = static_cast<float>(raw_.mouse_y) / static_cast<float>(config.mouse_max);
    mouse.wheel = raw_.mouse_z;
    mouse.left = raw_.mouse_left;
    mouse.right = raw_.mouse_right;
    mouse.middle = raw_.mouse_middle;

    // 解析键盘
    key.w = raw_.key & (1 << 0);
    key.s = raw_.key & (1 << 1);
    key.a = raw_.key & (1 << 2);
    key.d = raw_.key & (1 << 3);
    key.shift = raw_.key & (1 << 4);
    key.ctrl = raw_.key & (1 << 5);
    key.q = raw_.key & (1 << 6);
    key.e = raw_.key & (1 << 7);
    key.r = raw_.key & (1 << 8);
    key.f = raw_.key & (1 << 9);
    key.g = raw_.key & (1 << 10);
    key.z = raw_.key & (1 << 11);
    key.x = raw_.key & (1 << 12);
    key.c = raw_.key & (1 << 13);
    key.v = raw_.key & (1 << 14);
    key.b = raw_.key & (1 << 15);
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
    if (std::fabs(ret) < config.joystick_deadline)
        ret = 0.0f;
    return ret;
}
