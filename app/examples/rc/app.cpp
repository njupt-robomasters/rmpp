#include "bsp.hpp"
#include "rc/rc.hpp"
Rc rc({.uart = bsp.uart(3)}, {.uart = bsp.uart(6)});
#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            rc.on_loop();
        }
    }
}
