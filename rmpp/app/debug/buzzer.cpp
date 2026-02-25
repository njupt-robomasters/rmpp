#include "../infantry/misc.hpp"

void setup() {
    BSP::Init();
    buzzer.Play(Buzzer::C5D5G5);
}

void loop() {
    buzzer.OnLoop();
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