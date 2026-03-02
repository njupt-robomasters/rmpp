#pragma once

#include "bsp/bsp.hpp"
#include "unit/include_me.hpp"

class LED {
public:
    // LED配置
    struct config_t {
        UnitFloat<> period = 1 * s;         // 颜色循环周期
        UnitFloat<> saturation = 1 * ratio; // 饱和度
        UnitFloat<> brightness = 1 * ratio; // 亮度
    } config;

    LED(const config_t& config);

    // 需要在循环中调用
    void OnLoop();

private:
    UnitFloat<ratio> hue; // 当前色相
    BSP::Dwt dwt; // 用于计算dt
};