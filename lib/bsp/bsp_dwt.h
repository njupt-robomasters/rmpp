#pragma once

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

void BSP_DWT_Delay(float seconds);

float BSP_DWT_GetDT(uint32_t last_tick);

float BSP_DWT_UpdateDT(uint32_t* last_tick);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace BSP {
    class Dwt {
    private:
        uint32_t last_tick = 0;

    public:
        float dt = 0;
        float freq = 0;

        static void Init();

        static void Delay(float seconds);

        float GetDT() const;

        float UpdateDT();

        void Reset();
    };
}
#endif