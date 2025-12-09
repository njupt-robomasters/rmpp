#pragma once

class LED {
public:
    void OnLoop();

private:
    static constexpr float saturation = 1.0f; // 饱和度 (0-1)
    static constexpr float brightness = 1.0f; // 亮度 (0-1)
    static constexpr float hueStep = 360.0f * 0.001f; // 色相步进值

    float hue = 0.0f; // 初始色相 (0-360)
};