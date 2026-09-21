#pragma once

#include "board/board.hpp"

class Buzzer {
  public:
    static constexpr float C5D5G5[][2] = {
        {523, 250},
        {587, 250},
        {784, 500},
    };

    static constexpr float C5[][2] = {
        {523, 250},
    };

    static constexpr float C5C5C5[][2] = {
        {523, 250},
        {523, 250},
        {523, 250},
    };

    static constexpr float D5[][2] = {
        {587, 250},
    };

    static constexpr float D5D5D5[][2] = {
        {587, 250},
        {587, 250},
        {587, 250},
    };

    static constexpr float G5[][2] = {
        {784, 250},
    };

    static constexpr float G5G5G5[][2] = {
        {784, 250},
        {784, 250},
        {784, 250},
    };

    explicit Buzzer(rmpp::Pwm output) : output_(output) {}

    template <std::size_t N>
    void play(const float (&music)[N][2]) {
        this->music_ = music;
        i_ = 0;
        len_ = std::size(music);
        wait_play_ = true;
        timer_.reset();
    }

    void on_loop();

  private:
    rmpp::Pwm output_;
    const float (*music_)[2] = nullptr;
    uint32_t i_ = 0, len_ = 0;
    bool wait_play_ = false;
    rmpp::Timer timer_;
};
