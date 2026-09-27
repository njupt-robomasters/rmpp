#include "bsp/timer.hpp"
#include "imu.hpp"

#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            imu.update();
        }
    }
}
