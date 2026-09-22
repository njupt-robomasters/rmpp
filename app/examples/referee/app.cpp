#include "bsp.hpp"
#include "referee/referee.hpp"
Referee referee({.can = bsp.can(1)});
#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            referee.set_yaw(0 * deg, 0 * deg);
            referee.on_loop();
        }
    }
}
