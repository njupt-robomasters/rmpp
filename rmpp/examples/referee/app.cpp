#include "led.hpp"
#include "referee.hpp"

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();
    referee.OnLoop(0 * deg, 0 * deg);
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