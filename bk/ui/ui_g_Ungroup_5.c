//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_5.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 5

ui_string_frame_t ui_g_Ungroup_5;

ui_interface_string_t* ui_g_Ungroup_chassis_speed = &ui_g_Ungroup_5.option;

void _ui_init_g_Ungroup_5() {
    ui_g_Ungroup_5.option.figure_name[0] = FRAME_ID;
    ui_g_Ungroup_5.option.figure_name[1] = GROUP_ID;
    ui_g_Ungroup_5.option.figure_name[2] = START_ID;
    ui_g_Ungroup_5.option.operate_tpyel = 1;
    ui_g_Ungroup_5.option.figure_tpye = 7;
    ui_g_Ungroup_5.option.layer = 0;
    ui_g_Ungroup_5.option.font_size = 25;
    ui_g_Ungroup_5.option.start_x = 60;
    ui_g_Ungroup_5.option.start_y = 689;
    ui_g_Ungroup_5.option.color = 2;
    ui_g_Ungroup_5.option.str_length = 4;
    ui_g_Ungroup_5.option.width = 2;
    strcpy(ui_g_Ungroup_chassis_speed->string, "1m/s");

    ui_proc_string_frame(&ui_g_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_5, sizeof(ui_g_Ungroup_5));
}

void _ui_update_g_Ungroup_5() {
    ui_g_Ungroup_5.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_g_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_5, sizeof(ui_g_Ungroup_5));
}

void _ui_remove_g_Ungroup_5() {
    ui_g_Ungroup_5.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_g_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_5, sizeof(ui_g_Ungroup_5));
}