#pragma once

#include "motor/VESC.hpp"
#include "motor/M3508.hpp"

inline VESC rub_left({
    .can_port = 2,
    .master_id = 0x20,
    .slave_id = 0x20,
}, {});

inline VESC rub_right({
    .can_port = 2,
    .master_id = 0x21,
    .slave_id = 0x21,
}, {});