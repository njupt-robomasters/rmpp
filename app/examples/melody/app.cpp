/*
 * 蜂鸣器示例：普通模式播提示音，再用复音模式播放曲目
 * 换曲目改 include 和 play_poly 参数；合成占用用 SWD 观察 melody.monitor
 */

#include "bsp/timer.hpp"
#include "melody/melody.hpp"
#include "melody/music/always_with_me.hpp"
#include "melody/music/c5d5g5.hpp"
#include "melody/music/g5g5g5.hpp"
#include "melody/music/the_ordinary_road.hpp"
#include "melody/music/you.hpp"

static Melody melody({});

extern "C" void app_main() {
    melody.play(C5D5G5);
    // melody.play_poly(YOU);

    while (true) {
        // 1 kHz 更新
        static Timer timer;
        if (timer.elapsed() < 0.001f)
            continue;
        timer.restart();

        melody.update();
    }
}
