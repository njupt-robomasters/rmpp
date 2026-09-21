#pragma once

#include "board/board.hpp"
#include "lib/msg/mavlink.h"
#include "math/angle.hpp"
#include "math/conversions.hpp"
#include "referee/referee.hpp"
#include <cmath>

using math::Angle;
using math::ms;

class Mavlink {
  public:
    // Members use SI units (angles rad, angular speeds rad/s).
    // The wire protocol still uses degrees and rpm; conversion occurs in parse/send.
    struct Config {
        float timeout = 100 * ms; // 断联超时时间
        float send_freq = 1000;   // 发送频率
    } config;

    // 连接标志
    bool is_connect_cdc = false;
    bool is_connect_vision = false;
    bool is_connect_insta360 = false;
    bool is_connect_chassis_speed = false;
    bool is_connect_current_position = false;

    // 发送
    struct {
        Angle yaw;
        Angle pitch;
        Angle roll;
    } imu;

    // 发送
    struct {
        bool is_red = true;
        Referee::GameProgress game_progress = Referee::NOT_STARTED;
        float stage_remain_time{};
        float bullet_speed{};
    } referee;

    // 发送
    struct {
        float x{};
        float y{};
    } target_position;

    // 接收
    struct {
        bool is_detect = false;
        Angle yaw;
        Angle pitch;
        bool is_fire = false;
        uint8_t robot_id;
        float wr{};
        float distance{};
    } vision;

    // 接收
    struct {
        // Angles in rad; NaN means no detection. Confidence is dimensionless.
        float a0 = NAN, c0 = 0;
        float a1 = NAN, c1 = 0;
        float a2 = NAN, c2 = 0;
    } insta360;

    // 接收
    struct {
        float vx{};
        float vy{};
    } chassis_speed;

    // 接收
    struct {
        float x{};
        float y{};
        float yaw{};
    } current_position;

    Mavlink(const Config& config);

    // 需要在循环中调用
    void on_loop();

  private:
    static constexpr uint8_t SYSTEM_ID = 1;    // mavlink参数
    static constexpr uint8_t COMPONENT_ID = 1; // mavlink参数

    // 用于断联检测
    rmpp::Timer timer_cdc_, timer_auto_aim_, timer_insta360_, timer_chassis_speed_, timer_current_position_;

    // 用于控制发送频率
    rmpp::Timer timer_send_freq_;

    uint8_t idx_ = 0; // 报文分散在不同的时间点发送

    // CDC接收回调
    void callback(const uint8_t data[], uint32_t size);

    // 解析接收到的MAVLink消息
    void parse(const mavlink_message_t& msg);

    // 发送函数
    static void send(const mavlink_message_t& msg);
    void send_imu() const;
    void send_referee() const;
    void send_target_position() const;
};
