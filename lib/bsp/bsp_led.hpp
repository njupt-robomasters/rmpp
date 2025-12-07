#pragma once

namespace BSP {
    class LED {
    public:
        static void Init();

        static void SetRGB(float red, float green, float blue);

        static void SetHSV(float hue, float saturation, float brightness);
    };
}