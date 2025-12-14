#pragma once

#include "bsp/bsp.hpp"

class VT13 {
public:
    enum mode_e {
        C,
        N,
        S,
        ERR,
    };

    bool is_connected = false;

    // 遥控器操作
    float x = 0, y = 0, pitch = 0, yaw = 0, wheel = 0;
    mode_e mode = ERR;
    bool pause = false, trigger = false, fn_left = false, fn_right = false;

    // 键鼠操作
    int16_t mouse_yaw = 0, mouse_pitch = 0, mouse_z = 0;
    bool mouse_left = false, mouse_right = false, mouse_middle = false;

    struct {
        bool w = false, s = false, a = false, d = false;
        bool shift = false, ctrl = false;
        bool q = false, e = false;
        bool r = false, f = false, g = false;
        bool z = false, x = false, c = false;
        bool v = false, b = false;
    } key;

    BSP::Dwt dwt; // 用于断联超时检测

    VT13();

    void OnLoop();

private:
    static constexpr float TIMEOUT = 0.1f;         // 断联检测超时时间
    static constexpr float STICK_DEADLINE = 0.01f; // 摇杆死区，小于此值认为是0

    struct __attribute__((packed)) {
        uint8_t sof_1 : 8;   // 0xA9
        uint8_t sof_2 : 8;   // 0x53
        uint16_t ch_0 : 11;  // 右手水平
        uint16_t ch_1 : 11;  // 右手竖直
        uint16_t ch_2 : 11;  // 左手竖直
        uint16_t ch_3 : 11;  // 左手水平
        uint8_t mode_sw : 2; // 挡位切换开关，c - 0，n - 1，s - 2
        uint8_t pause : 1;   // 暂停按键
        uint8_t fn_1 : 1;    // 自定义按键（左）
        uint8_t fn_2 : 1;    // 自定义按键（右）
        uint16_t wheel : 11; // 拨轮
        uint8_t trigger : 1; // 扳机键

        uint8_t reserved_3bit : 3;

        int16_t mouse_x : 16;     // 鼠标X轴
        int16_t mouse_y : 16;     // 鼠标Y轴
        int16_t mouse_z : 16;     // 鼠标Z轴
        uint8_t mouse_left : 2;   // 鼠标左键
        uint8_t mouse_right : 2;  // 鼠标右键
        uint8_t mouse_middle : 2; // 鼠标中键

        uint8_t reserved_2bit : 2;

        uint16_t key : 16;   // 键盘按键
        uint16_t crc16 : 16; // CRC校验，CRC-16/CCITT-FALSE多项式 P(x) = x^16 + x^12 + x^5 + 1（对应0x1021），初始值0xFFFF，无输入输出反转，无异或
    } raw{};

    // 串口接收回调
    void callback(const uint8_t data[], uint16_t size);

    // 解析遥杆值，归一化到±1
    static float getStick(uint16_t value);

    void resetData();
};