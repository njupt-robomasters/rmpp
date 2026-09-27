#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"
#include "misc/buzzer.hpp"
Buzzer buzzer(Pwm(4, 3));
#include "math/unit.hpp"

extern "C" void app_main() {
    buzzer.play(Buzzer::C5D5G5);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            buzzer.update();
        }
    }
}
