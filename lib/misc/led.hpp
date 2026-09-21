#pragma once

#include "board/board.hpp"

class Led {
  public:
    // LED配置
    struct Config {
        float period = 1;        // 颜色循环周期
        float saturation = 1.0f; // 饱和度
        float brightness = 1.0f; // 亮度
    } config;

    Led(const Config& config);

    // 需要在循环中调用
    void on_loop();

  private:
    float hue_{};       // 当前色相
    rmpp::Timer timer_; // 用于计算dt
};