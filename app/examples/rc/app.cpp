#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "math/unit.hpp"
#include "rc/rc.hpp"

static FSi6X fsi6x({.uart = Uart(3)});
static VT13 vt13({.uart = Uart(6)});
static Rc rc({
                 .vxy_max = 1.0f,
                 .wr_max = 60 * rpm,
                 .axy = 100.0f,
                 .dxy = 100.0f,
                 .ar = 1000 * rpm,
                 .dr = 1000 * rpm,
                 .wyaw = 360 * deg_s,
                 .wpitch = 360 * deg_s,
                 .ayaw = 3600 * deg_s,
                 .apitch = 3600 * deg_s,
             },
             fsi6x, vt13);

extern "C" void app_main() {
    Timer timer;
    while (true) {
        if (timer.elapsed() > 1 * ms) {
            timer.restart();
            rc.update();
        }
    }
}
