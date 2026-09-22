#include "buzzer.hpp"
#include "math/unit.hpp"

void Buzzer::on_loop() {
    if (i_ >= len_) {
        output_.set_duty(0);
        return;
    }

    if (wait_play_) {
        const bool valid_note = output_.set_frequency(music_[i_][0]);
        output_.set_duty(valid_note ? 0.5f : 0.0f);
        wait_play_ = false;
    }

    if (timer_.get_dt() > music_[i_][1] * ms) {
        output_.set_duty(0);
        i_++;
        timer_.reset();
        wait_play_ = true;
    }
}
