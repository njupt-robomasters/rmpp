#pragma once

#include "bsp/bsp.hpp"
#include "lib/msg/mavlink.h"
#include "referee/Referee.hpp"

class Mavlink {
public:
    struct config_t {
        UnitFloat<> timeout = 200 * ms;   // 断联超时时间
        UnitFloat<> send_freq = 1000 * Hz; // 发送频率
    } config;

    // 连接标志
    bool is_connect_cdc = false;
    bool is_connect_vision = false;
    bool is_connect_insta360 = false;
    bool is_connect_chassis_speed = false;
    bool is_connect_current_position = false;

    // 发送
    struct {
        Angle<deg> yaw;
        Angle<deg> pitch;
        Angle<deg> roll;
    } imu;

    // 发送
    struct {
        bool is_red = true;
        Referee::game_progress_e game_progress = Referee::NOT_STARTED;
        UnitFloat<s> stage_remain_time;
        UnitFloat<m_s> bullet_speed;
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
    } vision;

    // 接收
    struct {
        float a0 = NAN, c0 = 0;
        float a1 = NAN, c1 = 0;
        float a2 = NAN, c2 = 0;
    } insta360;

    // 接收
    struct {
        UnitFloat<m_s> vx;
        UnitFloat<m_s> vy;
    } chassis_speed;

    // 接收
    struct {
        UnitFloat<m> x;
        UnitFloat<m> y;
        UnitFloat<deg> yaw;
    } current_position;

    Mavlink(const config_t& config);

    // 需要在循环中调用
    void OnLoop();

private:
    static constexpr uint8_t SYSTEM_ID = 1;    // mavlink参数
    static constexpr uint8_t COMPONENT_ID = 1; // mavlink参数

    // 用于断联检测
    BSP::Dwt dwt_cdc, dwt_auto_aim, dwt_insta360, dwt_chassis_speed, dwt_current_position;

    // 用于控制发送频率
    BSP::Dwt dwt_send_freq;

    uint8_t idx = 0; // 报文分散在不同的时间点发送

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