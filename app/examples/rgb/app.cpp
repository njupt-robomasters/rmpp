#include "bsp.hpp"
#include "rgb/rgb.hpp"
Rgb rgb({.red = bsp.pwm(5, 3), .green = bsp.pwm(5, 2), .blue = bsp.pwm(5, 1)});
#include "math/unit.hpp"

extern "C" void app_main() {

    Timer timer;
    while (true) {
        if (timer.poll_timeout(1 * ms)) {
            rgb.on_loop();
        }
    }
}
