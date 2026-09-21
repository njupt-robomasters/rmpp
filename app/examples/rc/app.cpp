#include "bsp.hpp"
#include "rc/rc.hpp"
Rc rc({}, {});
#include "math/conversions.hpp"

using math::ms;

extern "C" void app_main() {

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            rc.on_loop();
        }
    }
}
