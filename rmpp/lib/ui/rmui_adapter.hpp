#pragma once

#include "UI.hpp"
#include "referee/Referee.hpp"
#include "lib/ui.h"

// 适配rmui
extern "C" void print_message(const uint8_t* message, const int length) {
    UI::AddCanData(message, length);
}

inline void update_rmui_robot_id() {
    extern int ui_self_id;
    extern Referee referee;
    ui_self_id = referee.robot_id;
}

inline void (*ui_init_g[])() = {
    ui_init_g_00,
    ui_init_g_01,
    ui_init_g_02,
    ui_init_g_03,
    ui_init_g_04,
    ui_init_g_05,

    ui_init_g_1,
    ui_init_g_2,

    ui_init_g_30,
    ui_init_g_31,
    ui_init_g_32,
};

inline void (*ui_update_g[])() = {
    ui_update_g_1,
    ui_update_g_2,

    ui_update_g_1,
    ui_update_g_30,

    ui_update_g_1,
    ui_update_g_31,

    ui_update_g_1,
    ui_update_g_32,
};