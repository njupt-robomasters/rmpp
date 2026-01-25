#pragma once

#include "unit/include_me.hpp"

namespace BSP {
    class LED {
    public:
        // 初始化，需要调用一次
        static void Init();

        // 设置颜色（RGB格式）
        static void SetRGB(UnitFloat<> red, UnitFloat<> green, UnitFloat<> blue);

        // 设置颜色（HSV格式）
        static void SetHSV(UnitFloat<> hue, UnitFloat<> saturation, UnitFloat<> brightness);
    };
}