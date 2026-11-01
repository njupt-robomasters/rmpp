// 由 tools/midi_to_song.py 从 原单声部曲目 生成

#pragma once

#include "melody/melody.hpp"

inline constexpr Note C5D5G5_MELODY_0[] = {
    {72, 25},
    {74, 25},
    {79, 50},
};

inline constexpr Voice C5D5G5_MELODY[] = {C5D5G5_MELODY_0};

inline constexpr Song C5D5G5 = {.melody = C5D5G5_MELODY};
