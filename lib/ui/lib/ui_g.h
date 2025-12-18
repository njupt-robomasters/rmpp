//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[6];
extern uint8_t ui_g_dirty_figure[6];
extern ui_interface_string_t ui_g_now_strings[1];
extern uint8_t ui_g_dirty_string[1];

#define ui_g_Ungroup_hit ((ui_interface_arc_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_dir_round ((ui_interface_round_t*)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_dir ((ui_interface_line_t*)&(ui_g_now_figures[2]))
#define ui_g_Ungroup_cap_rect ((ui_interface_rect_t*)&(ui_g_now_figures[3]))
#define ui_g_Ungroup_cap ((ui_interface_rect_t*)&(ui_g_now_figures[4]))
#define ui_g_Ungroup_cap_text ((ui_interface_number_t*)&(ui_g_now_figures[5]))

#define ui_g_Ungroup_aim (&(ui_g_now_strings[0]))

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_hit_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_dir_round_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_dir_dirty (ui_g_dirty_figure[2])
#define ui_g_Ungroup_cap_rect_dirty (ui_g_dirty_figure[3])
#define ui_g_Ungroup_cap_dirty (ui_g_dirty_figure[4])
#define ui_g_Ungroup_cap_text_dirty (ui_g_dirty_figure[5])

#define ui_g_Ungroup_aim_dirty (ui_g_dirty_string[0])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
