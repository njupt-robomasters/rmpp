#pragma once

// 以下为C/C++均可见的内容
#ifdef __cplusplus
#include <cstdint>

extern "C" {
#else
#include <stdint.h>
#endif

void BSP_DWT_Delay(float seconds);

float BSP_DWT_GetDT(uint32_t last_tick);

float BSP_DWT_UpdateDT(uint32_t* last_tick);

#ifdef __cplusplus
}
#endif

// 以下为仅C++可见的内容
#ifdef __cplusplus

#include <unit/include_me.hpp>

namespace BSP {
    class Dwt {
    public:
        UnitFloat<> dt;
        UnitFloat<> freq;

        Dwt(const UnitFloat<>& min_dt = 1 * ms, const UnitFloat<>& max_dt = 10 * s);

        Dwt& operator=(const Dwt& other) {
            if (this != &other) {
                dt = other.dt;
                freq = other.freq;
                last_tick = other.last_tick;
                is_max_dt = other.is_max_dt;
            }
            return *this;
        }

        bool operator==(const Dwt& other) const {
            return last_tick == other.last_tick;
        }

        bool operator!=(const Dwt& other) const {
            return last_tick != other.last_tick;
        }

        static void Init();

        static void Delay(const UnitFloat<>& time);

        UnitFloat<> GetDT();

        UnitFloat<> UpdateDT();

        bool PollTimeout(const UnitFloat<>& time);

        void Reset();

    private:
        UnitFloat<> min_dt, max_dt;

        uint32_t last_tick;

        bool is_max_dt = false;
    };
}

#endif