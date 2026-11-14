#pragma once

#include "bsp/timer.hpp"
#include "bsp/uart.hpp"

class VT13 {
  public:
    struct Config {
        Uart uart;
        float timeout = 0.5f;     // 断联检测超时时间
        float deadline = 0.05f;   // 摇杆死区，小于此值认为是0
        uint16_t mouse_max = 500; // 鼠标xy最大速度
    };

    enum Mode {
        C,
        N,
        S,
        ERR,
    };

    bool is_connect = false;                                        // 连接标志
    float x = 0, y = 0, pitch = 0, yaw = 0;                         // 摇杆
    float wheel = 0;                                                // 左上角拨轮
    Mode mode = ERR;                                                // 模式切换开关
    bool pause = false, trigger = false, fn = false, photo = false; // 按钮

    // 鼠标
    struct Mouse {
        float yaw = 0, pitch = 0;
        int16_t wheel = 0;
        bool left = false, right = false, middle = false;
    } mouse;

    // 键盘
    struct Key {
        bool w = false, s = false, a = false, d = false;
        bool shift = false, ctrl = false;
        bool q = false, e = false, r = false, f = false, g = false;
        bool z = false, x = false, c = false, v = false, b = false;
    } key;

  private:
    const float timeout;      // 断联检测超时时间
    const float deadline;     // 摇杆死区，小于此值认为是0
    const uint16_t mouse_max; // 鼠标xy最大速度

    // 通道原始值
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

    uint8_t rx_buf[64]{};
    uint8_t rx_size = 0;
    Timer callback_timer; // 用于断联检测

  public:
    VT13(const Config& config);

    // 断联检测，需要在循环中调用；断联时数据复位为默认值
    void update();

  private:
    // 串口接收回调
    void callback(const uint8_t data[], size_t size);

    void parse_raw();

    void reset_data();

    // 解析摇杆值，归一化到±1
    float get_joy_stick(uint16_t value) const;
};
