#pragma once

#include "bsp/bsp.hpp"

class Buzzer {
public:
    static constexpr float C5D5G5[][2] = {
        {523, 250},
        {587, 250},
        {784, 500},
    };

    static constexpr float C5C5C5[][2] = {
        {523, 250},
        {523, 250},
        {523, 250},
    };

    static constexpr float D5D5D5[][2] = {
        {587, 250},
        {587, 250},
        {587, 250},
    };

    static constexpr float G5G5G5[][2] = {
        {784, 250},
        {784, 250},
        {784, 250},
    };

    Buzzer() = default;

    template <std::size_t N>
    void Play(const float (&music)[N][2]) {
        this->music = music;
        i = 0;
        len = std::size(music);
        wait_play = true;
        dwt.Reset();
    }

    void OnLoop();

private:
    const float (*music)[2] = nullptr;
    uint32_t i = 0, len = 0;
    bool wait_play = false;
    BSP::Dwt dwt;
};