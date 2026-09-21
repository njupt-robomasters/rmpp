#include "bsp.hpp"
#include "imu.hpp"

#include "math/conversions.hpp"

using math::ms;

extern "C" void app_main() {

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            imu.on_loop();
        }
    }
}
