#pragma once

#include "board/board.hpp" // timer
#include "math/conversions.hpp"

using math::ms;
using math::pct;

class FSi6X {
  public:
    struct Config {
        float timeout = 500 * ms;          // 断联检测超时时间
        float joystick_deadline = 5 * pct; // 摇杆死区，小于此值认为是0
    } config;

    enum Switch {
        DOWN,
        MID,
        UP,
        ERR
    };

    bool is_connect = false; // 连接标志

    float x{}, y{}, pitch{}, yaw{};                    // 摇杆
    float vra{}, vrb{};                                // 旋钮
    Switch swa = ERR, swb = ERR, swc = ERR, swd = ERR; // 拨杆

    FSi6X(const Config& config);

    void on_loop();

  private:
    // 通道原始值
    struct __attribute__((packed)) {
        uint8_t header : 8; // 0x0F

        // 摇杆范围：240(-784) ~ 1024 ~ 1807(+783)
        // 拨杆范围：UP:240(-784) ~ MID:1024 ~ DOWN:1807(+783)
        uint16_t ch0 : 11; // 右手水平
        uint16_t ch1 : 11; // 右手竖直
        uint16_t ch2 : 11; // 左手竖直
        uint16_t ch3 : 11; // 左手水平
        uint16_t ch4 : 11;
        uint16_t ch5 : 11;
        uint16_t ch6 : 11;
        uint16_t ch7 : 11;
        uint16_t ch8 : 11;
        uint16_t ch9 : 11;
        uint16_t ch10 : 11;
        uint16_t ch11 : 11;
        uint16_t ch12 : 11;
        uint16_t ch13 : 11;
        uint16_t ch14 : 11;
        uint16_t ch15 : 11;

        uint8_t is_disconnect : 8; // 连接上为0x00，断联为0x0C

        uint8_t tail : 8; // 0x00
    } raw_{};

    rmpp::Timer timer_connect_; // 用于断联检测

    // 串口接收回调
    void callback(const uint8_t data[], uint16_t size);

    // 解析摇杆值，归一化到±1
    float get_joystick(uint16_t value) const;

    // 解析拨杆值
    static Switch get_switch(uint16_t value);

    void reset_data();
};
