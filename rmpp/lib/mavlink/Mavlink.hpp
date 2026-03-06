#pragma once

#include "unit/include_me.hpp"
#include "bsp/bsp.hpp"
#include "lib/msg/mavlink.h"

class Mavlink {
public:
    bool is_connect = false;

    // 发送
    struct {
        Angle<deg> yaw;
        Angle<deg> pitch;
        Angle<deg> roll;
    } imu;

    // 发送
    struct {
        bool is_red = true;
        uint8_t game_progress = 0;
        uint16_t stage_remain_time = 0;
        UnitFloat<m_s> bullet_speed = 24.0f * m_s;
    } referee;

    // 发送
    struct {
        UnitFloat<m> x;
        UnitFloat<m> y;
    } target_position;

    // 接收
    struct {
        bool is_detect = false;
        Angle<deg> yaw;
        Angle<deg> pitch;
        bool is_fire = false;
        uint8_t robot_id;
        UnitFloat<rpm> wr;
        UnitFloat<m> distance;
    } auto_aim;

    // 接收
    struct {
        float x0 = 0, y0 = 0;
        float x1 = 0, y1 = 0;
        float x2 = 0, y2 = 0;
        float x3 = 0, y3 = 0;
    } ui;

    // 接收
    struct {
        UnitFloat<m> x;
        UnitFloat<m> y;
        UnitFloat<deg> yaw;
    } current_position;

    // 接收
    struct {
        UnitFloat<m_s> vx;
        UnitFloat<m_s> vy;
    } chassis_speed;

    Mavlink();

    // 需要在循环中调用
    void OnLoop();

private:
    static constexpr UnitFloat<> CONNECT_TIMEOUT = 100 * ms; // 断联检测超时时间
    static constexpr UnitFloat<> SEND_FREQ = 50 * Hz;
    static constexpr uint8_t SYSTEM_ID = 1;    // mavlink参数
    static constexpr uint8_t COMPONENT_ID = 1; // mavlink参数

    // 用于断联检测
    BSP::Dwt dwt_is_connect;

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
    void sendTargetPosition() const;
};