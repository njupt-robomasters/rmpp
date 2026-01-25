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

#include <unit/include_me.hpp>

namespace BSP {
    class Dwt {
    private:
        static constexpr UnitFloat<> MAX_DT = 10.0f * s;

        uint32_t last_tick = 0;
        bool is_max_dt = false;

    public:
        UnitFloat<> dt;
        UnitFloat<> freq;

        static void Init();

        static void Delay(const UnitFloat<>& time);

        UnitFloat<> GetDT();

        UnitFloat<> UpdateDT();

        bool PollTimeout(const UnitFloat<>& time);

        void Reset();
    };
}
#endif