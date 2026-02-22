#include "../led.hpp"
#include "../imu.hpp"

void setup() {
    BSP::Init();
    // imu.Calibrate();
}

void loop() {
    led.OnLoop();
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