#include "app.hpp"

static constexpr float saturation = 1.0f; // 饱和度 (0-1)
static constexpr float brightness = 1.0f; // 亮度 (0-1)
static constexpr float hueStep = 360.0f * 0.001f;

static float hue = 0.0f; // 初始色相 (0-360)

void loop_led() {
    // 更新色相值
    hue += hueStep;

    // 色相循环 (0-360)
    if (hue > 360.0f) {
        hue = 0.0f;
    }

    BSP::LED::SetHSV(hue, saturation, brightness);
}