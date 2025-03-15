#include "task_led.h"
#include <cstdint>
#include "cmsis_os.h"
#include "bsp_led.h"

#define FLOW_COLOR_LENGTH 6
#define FLOW_COLOR_CHANGE_TIME 1000

// 蓝 -> 绿(灭) -> 红 -> 蓝(灭) -> 绿 -> 红(灭)
constexpr uint32_t FLOW_COLOR_RGBA[FLOW_COLOR_LENGTH + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

[[noreturn]] void task_led_entry(void *argument) {
    while (true) {
        for (uint16_t i = 0; i < FLOW_COLOR_LENGTH; i++) {
            auto alpha = static_cast<float>((FLOW_COLOR_RGBA[i] & 0xFF000000) >> 24);
            auto red = static_cast<float>((FLOW_COLOR_RGBA[i] & 0x00FF0000) >> 16);
            auto green = static_cast<float>((FLOW_COLOR_RGBA[i] & 0x0000FF00) >> 8);
            auto blue = static_cast<float>((FLOW_COLOR_RGBA[i] & 0x000000FF) >> 0);

            float delta_alpha = static_cast<float>((FLOW_COLOR_RGBA[i + 1] & 0xFF000000) >> 24) - static_cast<float>((FLOW_COLOR_RGBA[i] & 0xFF000000) >> 24);
            float delta_red = static_cast<float>((FLOW_COLOR_RGBA[i + 1] & 0x00FF0000) >> 16) - static_cast<float>((FLOW_COLOR_RGBA[i] & 0x00FF0000) >> 16);
            float delta_green = static_cast<float>((FLOW_COLOR_RGBA[i + 1] & 0x0000FF00) >> 8) - static_cast<float>((FLOW_COLOR_RGBA[i] & 0x0000FF00) >> 8);
            float delta_blue = static_cast<float>((FLOW_COLOR_RGBA[i + 1] & 0x000000FF) >> 0) - static_cast<float>((FLOW_COLOR_RGBA[i] & 0x000000FF) >> 0);

            delta_alpha /= FLOW_COLOR_CHANGE_TIME;
            delta_red /= FLOW_COLOR_CHANGE_TIME;
            delta_green /= FLOW_COLOR_CHANGE_TIME;
            delta_blue /= FLOW_COLOR_CHANGE_TIME;

            for (uint16_t j = 0; j < FLOW_COLOR_CHANGE_TIME; j++) {
                alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;

                BSP_LED_ShowRGBA(red, green, blue, alpha);

                osDelay(1);
            }
        }
    }
}
