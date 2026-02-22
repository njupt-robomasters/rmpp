#include "../led.hpp"
#include "../referee.hpp"

void setup() {
    BSP::Init();
}

void loop() {
    led.OnLoop();

    referee.SetYaw(0 * deg, 0 * deg);
    referee.OnLoop();
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