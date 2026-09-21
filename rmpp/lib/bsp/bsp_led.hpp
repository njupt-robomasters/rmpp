#pragma once


namespace BSP {
    class LED {
    public:
        // 初始化，需要调用一次
        static void Init();

        // 设置颜色（RGB格式）
        static void SetRGB(float red, float green, float blue);

        // 设置颜色（HSV格式）
        static void SetHSV(float hue, float saturation, float brightness);
    };
}