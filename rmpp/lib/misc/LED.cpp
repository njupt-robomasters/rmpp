#include "LED.hpp"

LED::LED(const config_t& config) : config(config) {}

void LED::OnLoop() {
    // 计算dt
    const float dt = dwt.UpdateDT();

    // 更新色相值
    hue += dt;
    if (hue > 1)
        hue = 0.0f;

    BSP::LED::SetHSV(hue, config.saturation, config.brightness);
}