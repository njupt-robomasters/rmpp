#pragma once

#include <stdint.h>

#ifdef __cplusplus

namespace BSP {
    class Dwt {
        static uint64_t tick64; // 3481.8年溢出
        static uint32_t tick32; // 25.565秒溢出

        uint32_t tick = 0;

    public:
        float dt = 0;

        static void Init();

        static void Update();

        static float GetTime();

        static void Delay(float second);

        float GetDT();

        void Clear();
    };
}

extern "C" {
#endif

float BSP_DWT_GetDT(uint32_t* tick);

float BSP_DWT_GetTime();

void BSP_DWT_Delay(float second);

#ifdef __cplusplus
}
#endif
