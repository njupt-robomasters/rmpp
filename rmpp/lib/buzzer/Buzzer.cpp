#include "Buzzer.hpp"

void Buzzer::OnLoop() {
    if (i >= len) {
        BSP::BUZZER::Stop();
        return;
    }

    if (wait_play) {
        BSP::BUZZER::Play(music[i][0] * Hz);
        wait_play = false;
    }

    if (dwt.GetDT() > music[i][1] * ms) {
        BSP::BUZZER::Stop();
        i++;
        dwt.Reset();
        wait_play = true;
    }
}