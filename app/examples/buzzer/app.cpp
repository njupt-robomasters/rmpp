#include "bsp.hpp"
#include "misc/buzzer.hpp"
Buzzer buzzer(rmpp::board.pwm(4, 3));
#include "math/conversions.hpp"

using math::ms;

extern "C" void app_main() {
    buzzer.play(Buzzer::C5D5G5);

    rmpp::Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            buzzer.on_loop();
        }
    }
}
