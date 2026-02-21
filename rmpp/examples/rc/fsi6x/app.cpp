#include "led.hpp"
#include "rc.hpp"

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();
    fsi6x.OnLoop();
}

extern "C" void rmpp_main() {
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
        }
    }
}