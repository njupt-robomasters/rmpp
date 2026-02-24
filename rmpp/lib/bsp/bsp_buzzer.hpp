#pragma once

#include "unit/include_me.hpp"

namespace BSP {
    class BUZZER {
    public:
        // 初始化，需要调用一次
        static void Init();

        // 播放声音
        static void Play(const UnitFloat<>& freq);

        // 停止声音
        static void Stop();
    };
}