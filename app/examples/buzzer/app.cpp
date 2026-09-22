#include "bsp.hpp"
#include "misc/buzzer.hpp"
Buzzer buzzer(bsp.pwm(4, 3));
#include "math/unit.hpp"

extern "C" void app_main() {
    buzzer.play(Buzzer::C5D5G5);

    Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            buzzer.on_loop();
        }
    }
}
