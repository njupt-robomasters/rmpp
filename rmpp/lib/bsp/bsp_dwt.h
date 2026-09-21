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

#include "math/Conversions.hpp"

namespace BSP {
    class Dwt {
    public:
        float dt{};
        float freq{};

        Dwt(float min_dt = 0.1f * math::ms, float max_dt = 10.0f);

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

        static void Delay(float time);

        float GetDT();

        float UpdateDT();

        bool PollTimeout(float time);

        void Reset();

    private:
        float min_dt{}, max_dt{};

        uint32_t last_tick;

        bool is_max_dt = false;
    };
}

#endif
