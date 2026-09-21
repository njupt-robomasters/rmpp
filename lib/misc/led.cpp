#include "led.hpp"

Led::Led(const Config& config) : config(config) {}

void Led::on_loop() {
    // 计算dt
    const float dt = timer_.update_dt();

    // 更新色相值
    hue_ += dt;
    if (hue_ > 1)
        hue_ = 0.0f;

    rmpp::board.led(1).set_hsv(hue_, config.saturation, config.brightness);
}