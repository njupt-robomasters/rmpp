#include "bsp.hpp"
#include "imu.hpp"

#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            imu.on_loop();
        }
    }
}
