#include "referee/referee.hpp"

static Referee referee({.bus = Can(1)});

extern "C" void app_main() {
    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();
            referee.update();
        }
    }
}
