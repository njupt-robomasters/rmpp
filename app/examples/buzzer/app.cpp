#include "misc/buzzer.hpp"
#include "misc/music/always_with_me.hpp"

static Buzzer buzzer(Pwm(4, 3));

extern "C" void app_main() {
    buzzer.play(ALWAYS_WITH_ME);

    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();
            buzzer.update();
        }
    }
}
