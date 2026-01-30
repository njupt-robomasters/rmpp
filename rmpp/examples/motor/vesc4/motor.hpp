#pragma once

#include "motor/VESC.hpp"

inline VESC rub_left1({
                          .can_port = 2,
                          .master_id = 0x20,
                          .slave_id = 0x20,
                      }, {});

inline VESC rub_right1({
                           .can_port = 2,
                           .master_id = 0x21,
                           .slave_id = 0x21,
                       }, {});

inline VESC rub_left2({
                          .can_port = 2,
                          .master_id = 0x22,
                          .slave_id = 0x22,
                      }, {});

inline VESC rub_right2({
                           .can_port = 2,
                           .master_id = 0x23,
                           .slave_id = 0x23,
                       }, {});