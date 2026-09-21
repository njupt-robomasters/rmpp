#pragma once


namespace BSP {
    class BUZZER {
    public:
        // 初始化，需要调用一次
        static void Init();

        // 播放声音
        static void Play(float freq);

        // 停止声音
        static void Stop();
    };
}