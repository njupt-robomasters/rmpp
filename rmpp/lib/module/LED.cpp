#include "LED.hpp"

LED::LED(const config_t& config) : config(config) {}

void LED::OnLoop() {
    // 计算dt
    const UnitFloat dt = dwt.UpdateDT();

    // 更新色相值
    hue += dt / config.period;
    if (hue > 1) hue = 0 * ratio;

    BSP::LED::SetHSV(hue, config.saturation, config.brightness);
}