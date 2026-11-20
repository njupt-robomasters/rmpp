#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"

class Nav {
  private:
    static constexpr std::size_t FRAME_SIZE = 17;

  public:
    struct Config {
        float timeout = 100 * ms;
    };

  private:
    const Config config;

  public:
    bool is_connect = false;

    // vx/vy 为雷达所在大 yaw 系速度，m/s；yaw 为协议角度，rad，当前仅保存
    struct Command {
        float vx = 0;
        float vy = 0;
        float yaw = 0;
        uint8_t action_flag = 0; // 0 随头，1 小陀螺；pitch1 当前全程失能
    } command;

  private:
    Command pending;
    bool has_command = false;
    uint8_t rx_buf[FRAME_SIZE]{};
    std::size_t rx_size = 0;
    Timer callback_timer;

  public:
    Nav(Cdc cdc, const Config& config);

    // 主循环发布完整指令快照；超时清空输出并置为离线
    void update();

  private:
    void callback(const uint8_t* data, std::size_t size);
};
