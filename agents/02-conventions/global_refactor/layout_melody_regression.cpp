// 主机差分回归：直驱音符时序、带伴奏/纯旋律的复音采样与停止状态
#include "melody/melody.hpp"
#include <bit>
#include <cstdio>

static double sim_time = 0;
static Buzzer::Fill stream;
static unsigned tone_count = 0, mute_count = 0;

void Buzzer::set_tone(const float hz) const {
    tone_count++;
    std::printf("tone %.7g\n", hz);
}

void Buzzer::mute() const {
    mute_count++;
}

void Buzzer::start_stream(float, Fill fill) const {
    stream = fill;
}

void Buzzer::stop_stream() const {
    stream = {};
}

void Timer::delay(float) {}

float Timer::elapsed() const {
#ifdef LAYOUT_BASELINE
    return static_cast<float>(sim_time - last_cycles_ * 1e-6);
#else
    return static_cast<float>(sim_time - last_cycles * 1e-6);
#endif
}

float Timer::restart() {
    const float dt = elapsed();
#ifdef LAYOUT_BASELINE
    last_cycles_ = static_cast<uint64_t>(sim_time * 1e6);
#else
    last_cycles = static_cast<uint64_t>(sim_time * 1e6);
#endif
    return dt;
}

int main() {
    const Note melody_notes[] = {{69, 8}, {72, 4}, {0, 3}, {72, 5}};
    const Note chords_notes[] = {{57, 10}, {60, 10}};
    const Voice melody_voices[] = {melody_notes};
    const Voice chords_voices[] = {chords_notes};
    const Song songs[] = {{melody_voices, chords_voices}, {melody_voices, {}}};
    for (const auto& song : songs) {
        Melody player({});
        player.play(song);
        for (int i = 0; i < 500; i++) {
            sim_time += 0.001;
            player.update();
        }
        player.play_poly(song);
        for (int i = 0; i < 500; i++) {
            sim_time += 0.001;
            if (stream) {
                float duty[32];
                stream(duty);
                for (const float value : duty)
                    std::printf("%08x\n", std::bit_cast<unsigned>(value));
            }
            player.update();
        }
        std::printf("playing %d tones %u mutes %u\n", player.is_playing(), tone_count, mute_count);
    }
}
