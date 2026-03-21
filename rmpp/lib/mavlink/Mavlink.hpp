#pragma once

#include "bsp/bsp.hpp"
#include "lib/msg/mavlink.h"
#include "referee/Referee.hpp"

class Mavlink {
public:
    struct config_t {
        UnitFloat<> cdc_timeout = 200 * ms;     // CDC串口断联时间
        UnitFloat<> message_timeout = 200 * ms; // 每种报文断联时间
        UnitFloat<> send_freq = 1000 * Hz;      // 发送频率
    } config;

    // CDC串口连接标志
    bool is_connect_cdc = false;

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
    bool is_connect_auto_aim = false;

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
    bool is_connect_insta360 = false;

    struct {
        float a0 = NAN, c0 = 0;
        float a1 = NAN, c1 = 0;
        float a2 = NAN, c2 = 0;
    } insta360;

    // 接收
    bool is_connect_current_position = false;

    struct {
        UnitFloat<m> x;
        UnitFloat<m> y;
        UnitFloat<deg> yaw;
    } current_position;

    // 接收
    bool is_connect_chassis_speed = false;

    struct {
        UnitFloat<m_s> vx;
        UnitFloat<m_s> vy;
    } chassis_speed;

    Mavlink(const config_t& config);

    // 需要在循环中调用
    void OnLoop();

private:
    static constexpr uint8_t SYSTEM_ID = 1;    // mavlink参数
    static constexpr uint8_t COMPONENT_ID = 1; // mavlink参数

    BSP::Dwt dwt_cdc; // 用于断联检测
    BSP::Dwt dwt_auto_aim, dwt_insta360, dwt_current_position, dwt_chassis_speed;
    BSP::Dwt dwt_send_freq; // 用于控制发送频率

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