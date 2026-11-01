// 由 tools/midi_to_song.py 从 原单声部曲目 生成

#pragma once

#include "melody/melody.hpp"

inline constexpr Note G5G5G5_MELODY_0[] = {
    {79, 20},
    {0, 5},
    {79, 20},
    {0, 5},
    {79, 20},
};

inline constexpr Voice G5G5G5_MELODY[] = {G5G5G5_MELODY_0};

inline constexpr Song G5G5G5 = {.melody = G5G5G5_MELODY};
