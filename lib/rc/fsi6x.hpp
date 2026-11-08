#pragma once

#include "bsp/timer.hpp"
#include "bsp/uart.hpp"

class FSi6X {
  public:
    struct Config {
        Uart uart;
        float timeout = 0.5f;   // 断联检测超时时间
        float deadline = 0.05f; // 摇杆死区，小于此值认为是0
    };

    enum Switch {
        DOWN,
        MID,
        UP,
        ERR
    };

    bool is_connect = false; // 连接标志

    float x = 0, y = 0, pitch = 0, yaw = 0;            // 摇杆
    Switch swa = ERR, swb = ERR, swc = ERR, swd = ERR; // 拨杆
    float vra = 0, vrb = 0;                            // 旋钮

    FSi6X(const Config& config);

    // 断联检测，需要在循环中调用；断联时数据复位为默认值
    void update();

  private:
    const Config config_;

    // 通道原始值
    struct __attribute__((packed)) {
        uint8_t header : 8; // 0x0F

        // 摇杆范围：240(-784) ~ 1024 ~ 1807(+783)
        // 拨杆范围：UP:240(-784) ~ MID:1024 ~ DOWN:1807(+783)
        uint16_t ch0 : 11; // 右手水平，x
        uint16_t ch1 : 11; // 右手竖直，y
        uint16_t ch2 : 11; // 左手竖直，pitch
        uint16_t ch3 : 11; // 左手水平，yaw
        uint16_t ch4 : 11; // swa
        uint16_t ch5 : 11; // swb
        uint16_t ch6 : 11; // swc
        uint16_t ch7 : 11; // swd
        uint16_t ch8 : 11; // vra
        uint16_t ch9 : 11; // vrb
        uint16_t ch10 : 11;
        uint16_t ch11 : 11;
        uint16_t ch12 : 11;
        uint16_t ch13 : 11;
        uint16_t ch14 : 11;
        uint16_t ch15 : 11;

        uint8_t is_disconnect : 8; // 连接上为0x00，断联为0x0C

        uint8_t tail : 8; // 0x00
    } raw_{};

    Timer callback_timer_; // 用于断联检测

    // 串口接收回调
    void callback(const uint8_t data[], uint16_t size);

    void reset_data();

    // 解析摇杆值，归一化到±1
    float get_joystick(uint16_t value) const;

    // 解析拨杆值
    static Switch get_switch(uint16_t value);
};
