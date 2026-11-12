/*
 * 蜂鸣器示例：普通模式播提示音，再用复音模式播放曲目
 * 换曲目改 include 和 play_poly 参数；合成占用用 SWD 观察 melody.monitor
 */

#include "bsp/monitor.hpp"
#include "melody/melody.hpp"
#include "melody/music/always_with_me.hpp"
#include "melody/music/c5d5g5.hpp"
#include "melody/music/g5g5g5.hpp"
#include "melody/music/the_ordinary_road.hpp"
#include "melody/music/you.hpp"

static Melody melody({});

static void setup() {
    melody.play(C5D5G5);
    // melody.play(YOU);
    // melody.play_poly(YOU);
}

static void loop() {
    melody.update();
}

extern "C" void app_main() {
    setup();
    while (true) {
        // 1 kHz 更新
        static Timer app_timer;
        if (app_timer.elapsed() < 0.001f)
            continue;
        app_timer.restart();

        // 负载监控
        static Monitor app_monitor;
        app_monitor.begin();
        loop();
        app_monitor.end();
    }
}
