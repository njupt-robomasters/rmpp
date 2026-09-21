#include "../infantry/misc.hpp"
#include "math/Conversions.hpp"

using math::ms;

void setup() {
    BSP::Init();
}

void loop() {
    rc.OnLoop();
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