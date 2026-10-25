#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

static VT13 vt13({.uart = Uart(6)});
static FSi6X fsi6x({.uart = Uart(3)});

extern "C" void app_main() {
    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();
            vt13.update();
            fsi6x.update();
        }
    }
}
