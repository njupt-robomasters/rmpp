#include "vt13.hpp"
#include <cstring>
#include <algorithm>
#include "crc16.hpp"

VT13::VT13() {
    auto callback = std::bind(&VT13::callback,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2);
    BSP::UART3::RegisterCallback(callback);
}

void VT13::OnLoop() {
    if (dwt.GetDT() > TIMEOUT) {
        resetData();
    }
}

void VT13::callback(const uint8_t data[], const uint16_t size) {
    // 长度检查
    if (size != sizeof(raw)) return;

    // 帧头检查
    if (data[0] != 0xA9 || data[1] != 0x53) return;

    // CRC16校验
    const uint16_t crc16 = (data[sizeof(raw) - 2] << 8) | data[sizeof(raw) - 1];
    if (!CRC16::Verify(data, sizeof(raw) - 2, crc16)) return;

    std::memcpy(&raw, data, sizeof(raw));

    dwt.UpdateDT();
    is_connected = true;

    pitch = getStick(raw.ch_0);  // 右手垂直
    yaw = -getStick(raw.ch_1);   // 右手水平
    y = -getStick(raw.ch_2);     // 左手水平
    x = getStick(raw.ch_3);      // 左手垂直
    wheel = getStick(raw.wheel); // 拨轮

    mode = mode_e(raw.mode_sw);
    pause = raw.pause;
    trigger = raw.trigger;
    fn_left = raw.fn_1;
    fn_right = raw.fn_2;

    mouse_x = (float)raw.mouse_x / 32768.0f;
    mouse_y = (float)raw.mouse_y / 32768.0f;
    mouse_z = (float)raw.mouse_z / 32768.0f;
    mouse_left = raw.mouse_left;
    mouse_right = raw.mouse_right;
    mouse_middle = raw.mouse_middle;

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

float VT13::getStick(const uint16_t value) {
    float ret = ((float)value - 1024.0f) / 660.0f;
    ret = std::clamp(ret, -1.0f, 1.0f);
    if (std::abs(ret) < STICK_DEADLINE) ret = 0;
    return ret;
}

void VT13::resetData() {
    is_connected = false;

    // 遥控器操作
    x = y = pitch = yaw = wheel = 0;
    mode = ERR;
    pause = trigger = fn_left = fn_right = false;

    // 键鼠操作
    mouse_x = mouse_y = mouse_z = 0;
    mouse_left = mouse_right = mouse_middle = false;
    key = {};
}