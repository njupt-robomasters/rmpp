#include "misc/melody.hpp"
#include "misc/music/always_with_me.hpp"

static Melody melody;

extern "C" void app_main() {
    melody.play(ALWAYS_WITH_ME);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();
            melody.update();
        }
    }
}
