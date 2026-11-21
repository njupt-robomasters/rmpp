#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include <cmath>

class Aim {
  private:
    static constexpr std::size_t RX_FRAME_SIZE = 18, TX_FRAME_SIZE = 21;
    static constexpr float SEND_PERIOD = 1.0f / 200; // 5 ms

  public:
    struct Config {
        float timeout = 200 * ms; // 按最后一帧有效视觉数据计时
    };

  private:
    const Cdc cdc;
    const Config config;

    struct {
        bool is_red = true;
        float bullet_speed = 0;
    } referee;

  public:
    bool is_connect_vision = false;

    // 最终 IMU 头的世界系目标；内部使用弧度，pitch 为绕 Y 轴的右手方向
    struct Vision {
        bool is_detect = false;
        Angle yaw;
        Angle pitch;
        bool is_fire = false;
        float distance = 0;
    } vision;

    struct {
        float a0 = NAN;
    } insta360;

  private:
    Vision pending;
    bool has_vision = false;
    uint8_t rx_buf[RX_FRAME_SIZE]{};
    std::size_t rx_size = 0;
    Timer send_timer, callback_timer;

  public:
    Aim(Cdc cdc, const Config& config);

    void set_referee(bool is_red, float bullet_speed);

    // 输入最终头部的最新 IMU 姿态，rad；发布视觉快照并以 200 Hz 上报
    void update(Angle yaw, Angle pitch, Angle roll);

  private:
    void callback(const uint8_t* data, std::size_t size);
};
