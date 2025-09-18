#include "app.hpp"

static constexpr float saturation = 1.0f; // 饱和度 (0-1)
static constexpr float brightness = 1.0f; // 亮度 (0-1)
static constexpr float hueStep = 360.0f * 0.001f;

extern "C" void task_led_entry(const void* argument) {
    float hue = 0.0f; // 初始色相 (0-360)
    while (true) {
        // 更新色相值
        hue += hueStep;

        // 色相循环 (0-360)
        if (hue >= 360.0f) {
            hue = 0.0f;
        }

        BSP::LED::SetHSV(hue, saturation, brightness);

        BSP::OS::Delay(1);
    }
}
