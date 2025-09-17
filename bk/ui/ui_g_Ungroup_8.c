//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_8.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 8

ui_string_frame_t ui_g_Ungroup_8;

ui_interface_string_t* ui_g_Ungroup_super_cap_status = &ui_g_Ungroup_8.option;

void _ui_init_g_Ungroup_8() {
    ui_g_Ungroup_8.option.figure_name[0] = FRAME_ID;
    ui_g_Ungroup_8.option.figure_name[1] = GROUP_ID;
    ui_g_Ungroup_8.option.figure_name[2] = START_ID;
    ui_g_Ungroup_8.option.operate_tpyel = 1;
    ui_g_Ungroup_8.option.figure_tpye = 7;
    ui_g_Ungroup_8.option.layer = 0;
    ui_g_Ungroup_8.option.font_size = 30;
    ui_g_Ungroup_8.option.start_x = 1650;
    ui_g_Ungroup_8.option.start_y = 400;
    ui_g_Ungroup_8.option.color = 2;
    ui_g_Ungroup_8.option.str_length = 7;
    ui_g_Ungroup_8.option.width = 3;
    strcpy(ui_g_Ungroup_super_cap_status->string, "Cap: 0%");

    ui_proc_string_frame(&ui_g_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_8, sizeof(ui_g_Ungroup_8));
}

void _ui_update_g_Ungroup_8() {
    ui_g_Ungroup_8.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_8, sizeof(ui_g_Ungroup_8));
}

void _ui_remove_g_Ungroup_8() {
    ui_g_Ungroup_8.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_8, sizeof(ui_g_Ungroup_8));
}