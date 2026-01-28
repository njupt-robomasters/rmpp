#include "VT13.hpp"
#include <cstring> // memcpy
#include "referee/lib/crc.hpp"

VT13::VT13(const config_t& config) : RC(config) {
    // 注册串口回调
    auto callback = [this](const uint8_t data[], const uint16_t size) {
        this->callback(data, size);
    };
    BSP::UART6::RegisterCallback(callback);
}

void VT13::OnLoop() {
    // 断联检测
    if (dwt_connect.GetDT() > config.connect_timeout) {
        resetData();
    }
}

void VT13::callback(const uint8_t data[], const uint16_t size) {
    // 长度检查
    if (size != sizeof(raw)) return;

    // 帧头检查
    if (data[0] != 0xA9 || data[1] != 0x53) return;

    // CRC16校验
    const uint16_t crc16 = data[sizeof(raw) - 2] | (data[sizeof(raw) - 1] << 8);
    if (!CRC16::Verify(data, sizeof(raw) - 2, crc16)) return;

    // 复位断联检测
    is_connect = true;
    dwt_connect.UpdateDT();

    // 解析原始通道值
    std::memcpy(&raw, data, sizeof(raw));

    // 遥控器摇杆
    y = -getJoyStick(raw.ch_0);      // 右手水平
    x = getJoyStick(raw.ch_1);       // 右手垂直
    pitch = getJoyStick(raw.ch_2);   // 左手垂直
    yaw = -getJoyStick(raw.ch_3);    // 左手水平
    wheel = -getJoyStick(raw.wheel); // 拨轮

    // 遥控器开关和按钮
    mode = mode_e(raw.mode_sw);
    pause = raw.pause;
    trigger = raw.trigger;
    fn_left = raw.fn_1;
    fn_right = raw.fn_2;

    // 鼠标
    mouse.yaw = -(float)raw.mouse_x / (float)MOUSE_MAX * ratio;
    mouse.pitch = (float)raw.mouse_y / (float)MOUSE_MAX * ratio;
    mouse.wheel = raw.mouse_z;
    mouse.left = raw.mouse_left;
    mouse.right = raw.mouse_right;
    mouse.middle = raw.mouse_middle;

    // 键盘
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

    // 保护模式
    if (mode == N || mode == S) {
        is_protect = false;
    } else {
        is_protect = true;
    }
}

UnitFloat<> VT13::getJoyStick(const uint16_t value) const {
    UnitFloat<> ret = ((float)value - 1024.0f) / 660.0f * ratio;
    ret = unit::clamp(ret, -1 * ratio, 1 * ratio);
    if (unit::abs(ret) < config.joystick_deadline) ret = 0 * ratio;
    return ret;
}

void VT13::resetData() {
    is_connect = false;
    is_protect = true;

    // 遥控器
    x = y = pitch = yaw = 0 * ratio;
    wheel = 0 * ratio;
    mode = ERR;
    pause = trigger = fn_left = fn_right = false;

    // 键鼠
    mouse = {};
    key = {};
}