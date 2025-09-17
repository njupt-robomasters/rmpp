#pragma once

#include <cstdint>

class DJ6 {
public:
    enum switch_e {
        ERR,
        DOWN,
        MID,
        UP
    };

private:
    // 摇杆读数范围
    static constexpr int STICK_MAX = 660; // 摇杆最大值，超过此值钳位
    static constexpr float STICK_DEADLINE = 0.01f; // 摇杆死区，小于此值认为是0

    static constexpr int SBUS_FRAME_SIZE = 25; // DBUS帧长度

    struct {
        bool is_connected = false; // 是否连接上遥控器
        uint16_t CH1 = 1024; // 右手水平 364~1024~1684 (1024±660)
        uint16_t CH2 = 1024; // 右手竖直
        uint16_t CH3 = 1024; // 左手竖直
        uint16_t CH4 = 1024; // 左手水平
        uint16_t CH5 = 1024; // 背面拨杆
        uint16_t CH6 = 1024; // 左拨杆 511~1024~1541 (1024-513, 1024+517)
        uint16_t CH7 = 1024; // 右拨杆
        uint16_t CH8 = 1024;
        uint16_t CH9 = 1024;
        uint16_t CH10 = 1024;
        uint16_t CH11 = 1024;
        uint16_t CH12 = 1024;
        uint16_t CH13 = 1024;
        uint16_t CH14 = 1024;
        uint16_t CH15 = 1024;
        uint16_t CH16= 1024;
    } raw{};

    void callback(const uint8_t data[], uint16_t size);

    void parseSBUS(const uint8_t* data);

    float get_stick(uint16_t val);

    switch_e get_switch(uint16_t val);

public:
    bool is_connected = false;
    float x = 0, y = 0;
    float pitch = 0, yaw = 0;
    switch_e left_switch = ERR;
    switch_e right_switch = ERR;

    DJ6();
};
