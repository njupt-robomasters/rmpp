//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_7.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 7

ui_string_frame_t ui_g_Ungroup_7;

ui_interface_string_t* ui_g_Ungroup_enemy_gain_status = &ui_g_Ungroup_7.option;

void _ui_init_g_Ungroup_7() {
    ui_g_Ungroup_7.option.figure_name[0] = FRAME_ID;
    ui_g_Ungroup_7.option.figure_name[1] = GROUP_ID;
    ui_g_Ungroup_7.option.figure_name[2] = START_ID;
    ui_g_Ungroup_7.option.operate_tpyel = 1;
    ui_g_Ungroup_7.option.figure_tpye = 7;
    ui_g_Ungroup_7.option.layer = 0;
    ui_g_Ungroup_7.option.font_size = 25;
    ui_g_Ungroup_7.option.start_x = 1313;
    ui_g_Ungroup_7.option.start_y = 755;
    ui_g_Ungroup_7.option.color = 3;
    ui_g_Ungroup_7.option.str_length = 10;
    ui_g_Ungroup_7.option.width = 2;
    strcpy(ui_g_Ungroup_enemy_gain_status->string, "Enemy_gain");

    ui_proc_string_frame(&ui_g_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_7, sizeof(ui_g_Ungroup_7));
}
void _ui_update_g_Ungroup_7() {
    ui_g_Ungroup_7.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_7, sizeof(ui_g_Ungroup_7));
}

void _ui_remove_g_Ungroup_7() {
    ui_g_Ungroup_7.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_7, sizeof(ui_g_Ungroup_7));
}