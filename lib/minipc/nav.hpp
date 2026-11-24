#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"

class Nav {
  private:
    static constexpr std::size_t FRAME_SIZE = 17, TX_FRAME_SIZE = 8;
    static constexpr float SEND_PERIOD = 1.0f / 200; // 5 ms

  public:
    struct Config {
        float timeout = 100 * ms;
    };

  private:
    const Cdc cdc;
    const Config config;

  public:
    bool is_connect = false;

    // vx/vy 为最终 IMU 头坐标系速度，m/s；yaw 为协议角度，rad，当前仅保存
    struct Command {
        float vx = 0;
        float vy = 0;
        float yaw = 0;
        uint8_t action_flag = 0; // 0 小陀螺，1 随头；pitch1 当前全程失能
    } command;

  private:
    Command pending;
    bool has_command = false;
    uint8_t rx_buf[FRAME_SIZE]{};
    std::size_t rx_size = 0;
    Timer send_timer, callback_timer;

  public:
    Nav(Cdc cdc, const Config& config);

    // 发布导航指令；大 yaw 在线时以 200 Hz 上报相对底盘角，rad，[-pi, pi)
    // 无参数调用仅接收导航指令
    void update(float yaw = 0, bool is_yaw_connect = false);

  private:
    void callback(const uint8_t* data, std::size_t size);
};
