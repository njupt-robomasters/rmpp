#include "bsp.hpp"
#include "misc/led.hpp"
Led led({});
#include "math/conversions.hpp"

using math::ms;

extern "C" void app_main() {

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            led.on_loop();
        }
    }
}
