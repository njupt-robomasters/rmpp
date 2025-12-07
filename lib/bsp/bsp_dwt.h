#pragma once

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

float BSP_DWT_GetDT(uint32_t * last_tick);

float BSP_DWT_GetTime();

void BSP_DWT_Delay(float seconds);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace BSP {
    class Dwt {
    private:
        static uint64_t tick64; // 3481.8年溢出
        static uint32_t tick32; // 25.565秒溢出

        uint32_t last_tick = 0;

        static void onLoop();

    public:
        float dt = 0;
        float freq = 0;

        static void Init();

        static float GetTime();

        static void Delay(float seconds);

        float GetDT();

        void Update();

        void Clear();
    };
}
#endif