#pragma once

#include <cstdint>

class DJ6 {
public:
    typedef enum {
        DOWN,
        MID,
        UP
    } switch_t;

    bool is_connected = false;
    float x = 0, y = 0;
    float pitch = 0, yaw = 0;
    switch_t left_switch = DOWN;
    switch_t right_switch = DOWN;

    void ParseData(const uint8_t *data, uint16_t size);

private:
    // 摇杆范围
    static constexpr int RC_XY_MIN = 10; // 遥控器摇杆最小值，低于此值输出0
    static constexpr int RC_XY_MAX = 660; // 遥控器摇杆最大值，超过此值钳位

    static constexpr int DBUS_FRAME_SIZE = 25; // DBUS帧长度

    typedef struct {
        bool is_connected; // 是否连接上遥控器
        uint16_t CH1; // 右手水平 364~1024~1684 (1024±660)
        uint16_t CH2; // 右手竖直
        uint16_t CH3; // 左手竖直
        uint16_t CH4; // 左手水平
        uint16_t CH5; // 背面拨杆
        uint16_t CH6; // 左拨杆 511~1024~1541 (1024-513, 1024+517)
        uint16_t CH7; // 右拨杆
        uint16_t CH8;
        uint16_t CH9;
        uint16_t CH10;
        uint16_t CH11;
        uint16_t CH12;
        uint16_t CH13;
        uint16_t CH14;
        uint16_t CH15;
        uint16_t CH16;
    } channel_t;

    channel_t channel{};

    void parseDBUS(const uint8_t *data);

    static float norm_xy(uint16_t val);

    static switch_t get_switch(uint16_t val);
};
