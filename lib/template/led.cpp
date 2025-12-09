#include "led.hpp"
#include "bsp/bsp.hpp"

void LED::OnLoop() {
    // 更新色相值
    hue += hueStep;

    // 色相循环 (0-360)
    if (hue > 360.0f) {
        hue = 0.0f;
    }

    BSP::LED::SetHSV(hue, saturation, brightness);
}
