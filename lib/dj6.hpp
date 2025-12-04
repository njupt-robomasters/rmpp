#pragma once

#include "bsp/bsp.hpp"

class DJ6 {
private:
    // 用于断联检测
    template <typename T>
    class Wrapper {
    public:
        Wrapper(T value, DJ6* parent) : value(value), parent(parent) {}

        // 取值运算
        operator T() const {
            if (BSP::Dwt::GetTime() - parent->last_receive_time > TIMEOUT) {
                parent->resetData();
            }
            return value;
        }

        // 赋值运算
        Wrapper& operator=(const T& value) {
            this->value = value;
            return *this;
        }

    private:
        T value;
        DJ6* parent; // 保存父类指针
    };

public:
    static constexpr float STICK_DEADLINE = 0.01f; // 摇杆死区，小于此值认为是0
    static constexpr float TIMEOUT = 0.1f;         // 断联检测超时时间

    enum switch_e {
        ERR,
        DOWN,
        MID,
        UP
    };

    struct {
        bool is_connected = false; // 是否连接上遥控器
        uint16_t CH1 = 1024;       // 右手水平 364~1024~1684 (1024±660)
        uint16_t CH2 = 1024;       // 右手竖直
        uint16_t CH3 = 1024;       // 左手竖直
        uint16_t CH4 = 1024;       // 左手水平
        uint16_t CH5 = 1024;       // 背面拨杆
        uint16_t CH6 = 1024;       // 左拨杆 511~1024~1541 (1024-513, 1024+517)
        uint16_t CH7 = 1024;       // 右拨杆
        uint16_t CH8 = 1024;
        uint16_t CH9 = 1024;
        uint16_t CH10 = 1024;
        uint16_t CH11 = 1024;
        uint16_t CH12 = 1024;
        uint16_t CH13 = 1024;
        uint16_t CH14 = 1024;
        uint16_t CH15 = 1024;
        uint16_t CH16 = 1024;
    } raw;

    float last_receive_time = 0;

    bool is_connected = false;
    Wrapper<float> x, y, pitch, yaw;
    Wrapper<switch_e> left_switch, right_switch;

    DJ6();

private:
    // 串口接收回调
    void callback(const uint8_t data[], uint16_t size);

    // 解析SBUS报文
    void parseSBUS(const uint8_t* data);

    // 解析遥杆值，归一化到±1
    static float getStick(uint16_t value);

    // 解析拨杆值
    static switch_e getSwitch(uint16_t value);

    // 复位所有摇杆、拨杆值
    void resetData();
};
