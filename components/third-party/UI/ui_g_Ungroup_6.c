//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_6.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 6

ui_string_frame_t ui_g_Ungroup_6;
ui_interface_string_t* ui_g_Ungroup_our_gain_status = &ui_g_Ungroup_6.option;
void _ui_init_g_Ungroup_6() {
    ui_g_Ungroup_6.option.figure_name[0] = FRAME_ID;
    ui_g_Ungroup_6.option.figure_name[1] = GROUP_ID;
    ui_g_Ungroup_6.option.figure_name[2] = START_ID;
    ui_g_Ungroup_6.option.operate_tpyel = 1;
    ui_g_Ungroup_6.option.figure_tpye = 7;
    ui_g_Ungroup_6.option.layer = 0;
    ui_g_Ungroup_6.option.font_size = 25;
    ui_g_Ungroup_6.option.start_x = 1313;
    ui_g_Ungroup_6.option.start_y = 807;
    ui_g_Ungroup_6.option.color = 2;
    ui_g_Ungroup_6.option.str_length = 8;
    ui_g_Ungroup_6.option.width = 2;
    strcpy(ui_g_Ungroup_our_gain_status->string, "our_Gain");
    ui_proc_string_frame(&ui_g_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_6, sizeof(ui_g_Ungroup_6));
}

void _ui_update_g_Ungroup_6() {
    ui_g_Ungroup_6.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_6, sizeof(ui_g_Ungroup_6));
}

void _ui_remove_g_Ungroup_6() {
    ui_g_Ungroup_6.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_6, sizeof(ui_g_Ungroup_6));
}