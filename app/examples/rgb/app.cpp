#include "bsp/pwm.hpp"
#include "bsp/timer.hpp"
#include "misc/rgb.hpp"
Rgb rgb({.red = Pwm(5, 3), .green = Pwm(5, 2), .blue = Pwm(5, 1)});
#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            rgb.update();
        }
    }
}
