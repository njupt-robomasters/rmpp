#include "bsp.hpp"
#include "referee/referee.hpp"
Referee referee({});
#include "math/conversions.hpp"

using math::deg;
using math::ms;

extern "C" void app_main() {

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            referee.set_yaw(0 * deg, 0 * deg);
            referee.on_loop();
        }
    }
}
