#pragma once

#include "bsp/timer.hpp"
#include "bsp/uart.hpp"

class FSi6X {
  public:
    struct Config {
        Uart uart;
        float timeout = 0.5f;    // 断联超时，s
        float dead_zone = 0.05f; // 摇杆死区，归一化值；绝对值小于此值时为 0
    } const config;

    enum Switch {
        DOWN,
        MID,
        UP,
        ERR
    };

    bool is_connect = false;

    // 操控量，接收回调解析，断联或接收机失控时复位
    float x = 0, y = 0, pitch = 0, yaw = 0;            // 摇杆，±1；x 上推为正，y 左推为正
    Switch swa = ERR, swb = ERR, swc = ERR, swd = ERR; // 拨杆

  private:
    float vra = 0, vrb = 0; // 旋钮，±1

    // SBUS 原始帧，最近一帧数据；断联后保留供 SWD 查看
    struct __attribute__((packed)) {
        uint8_t header : 8; // 0x0F

        // 摇杆范围：240(-784) ~ 1024 ~ 1807(+783)
        // 拨杆范围：UP:240(-784) ~ MID:1024 ~ DOWN:1807(+783)
        uint16_t ch0 : 11; // 右手水平，y
        uint16_t ch1 : 11; // 右手竖直，x
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
    } raw{};

    Timer callback_timer; // 用于断联检测

  public:
    explicit FSi6X(const Config& config);

    // 每周期调用：断联检测，断联时操控量复位
    void update();

  private:
    void callback(const uint8_t* data, size_t size);

    void parse_raw();

    // 摇杆原始值 240~1024~1807 归一化到 ±1，死区内为 0
    float get_joystick(uint16_t value) const;

    // 拨杆原始值 240 上、1024 中、1807 下，其他为 ERR
    static Switch get_switch(uint16_t value);

    void reset();
};
