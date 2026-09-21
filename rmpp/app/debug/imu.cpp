#include "../infantry/imu.hpp"
#include "../infantry/misc.hpp"
#include "math/Conversions.hpp"

using math::ms;

void setup() {
    BSP::Init();
    // imu.Calibrate();
}

void loop() {
    imu.OnLoop();
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