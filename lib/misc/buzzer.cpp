#include "buzzer.hpp"
#include "math/unit.hpp"

void Buzzer::update() {
    if (i_ >= len_) {
        output_.set_duty(0);
        return;
    }

    if (wait_play_) {
        const bool valid_note = output_.set_frequency(music_[i_][0]);
        output_.set_duty(valid_note ? 0.5f : 0.0f);
        wait_play_ = false;
    }

    if (timer_.elapsed() > music_[i_][1] * ms) {
        timer_.restart();
        output_.set_duty(0);
        i_++;
        wait_play_ = true;
    }
}
