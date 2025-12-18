#pragma once

#include "unit/include_me.hpp"
#include "bsp/bsp.hpp"
#include "lib/msg/mavlink.h"

class Mavlink {
public:
    bool is_connected = false;

    // 发送
    struct {
        Angle<deg> yaw;
        Angle<deg> pitch;
        Angle<deg> roll;
    } imu;

    // 发送
    struct {
        bool is_red = true;
        UnitFloat<m_s> bullet_speed = 24.0f * m_s;
    } referee;

    // 接收
    struct {
        bool is_detected = false;
        bool is_fire_advise = false;
        Angle<deg> yaw;
        Angle<deg> pitch;
    } aim;

    // 接收
    struct {
        uint8_t num = 0;
        UnitFloat<rpm> speed;
        float x0 = 0, y0 = 0;
        float x1 = 0, y1 = 0, x2 = 0, y2 = 0, x3 = 0, y3 = 0, x4 = 0, y4 = 0;
    } ui;

    Mavlink();

    // 需要在循环中调用
    void OnLoop();

private:
    static constexpr float CONNECT_TIMEOUT = 0.1f; // 断联检测超时时间
    static constexpr UnitFloat SEND_FREQ = 100.0f * Hz;
    static constexpr uint8_t SYSTEM_ID = 1;    // mavlink参数
    static constexpr uint8_t COMPONENT_ID = 1; // mavlink参数

    // 用于断联检测
    BSP::Dwt dwt_is_connected;

    // 用于控制发送频率
    BSP::Dwt dwt_send_freq;

    // CDC接收回调
    void callback(const uint8_t data[], uint32_t size);

    // 解析接收到的MAVLink消息
    void parse(const mavlink_message_t& msg);

    // 发送函数
    static void send(const mavlink_message_t& msg);
    void sendImu() const;
    void sendReferee() const;
};