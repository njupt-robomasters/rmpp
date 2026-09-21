#pragma once

#include "bsp/bsp.hpp"

class LED {
public:
    // LED配置
    struct config_t {
        float period = 1;        // 颜色循环周期
        float saturation = 1.0f; // 饱和度
        float brightness = 1.0f; // 亮度
    } config;

    LED(const config_t& config);

    // 需要在循环中调用
    void OnLoop();

private:
    float hue{};  // 当前色相
    BSP::Dwt dwt; // 用于计算dt
};