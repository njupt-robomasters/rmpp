#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "referee/referee.hpp"
Referee referee({.can = Can(1)});
#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            referee.set_yaw(0 * deg, 0 * deg);
            referee.update();
        }
    }
}
