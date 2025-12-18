//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_Ungroup_0;

ui_interface_round_t *ui_g_Ungroup_dir_round = (ui_interface_round_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_dir = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_arc_t *ui_g_Ungroup_hit = (ui_interface_arc_t*)&(ui_g_Ungroup_0.data[2]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_dir_round->figure_type = 2;
    ui_g_Ungroup_dir_round->operate_type = 1;
    ui_g_Ungroup_dir_round->layer = 0;
    ui_g_Ungroup_dir_round->color = 2;
    ui_g_Ungroup_dir_round->start_x = 960;
    ui_g_Ungroup_dir_round->start_y = 250;
    ui_g_Ungroup_dir_round->width = 3;
    ui_g_Ungroup_dir_round->r = 50;

    ui_g_Ungroup_dir->figure_type = 0;
    ui_g_Ungroup_dir->operate_type = 1;
    ui_g_Ungroup_dir->layer = 0;
    ui_g_Ungroup_dir->color = 2;
    ui_g_Ungroup_dir->start_x = 960;
    ui_g_Ungroup_dir->start_y = 250;
    ui_g_Ungroup_dir->width = 3;
    ui_g_Ungroup_dir->end_x = 960;
    ui_g_Ungroup_dir->end_y = 300;

    ui_g_Ungroup_hit->figure_type = 4;
    ui_g_Ungroup_hit->operate_type = 1;
    ui_g_Ungroup_hit->layer = 0;
    ui_g_Ungroup_hit->color = 1;
    ui_g_Ungroup_hit->start_x = 960;
    ui_g_Ungroup_hit->start_y = 540;
    ui_g_Ungroup_hit->width = 0;
    ui_g_Ungroup_hit->start_angle = 0;
    ui_g_Ungroup_hit->end_angle = 0;
    ui_g_Ungroup_hit->rx = 360;
    ui_g_Ungroup_hit->ry = 360;


    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

ui_string_frame_t ui_g_Ungroup_1;
ui_interface_string_t* ui_g_Ungroup_aim = &(ui_g_Ungroup_1.option);

void _ui_init_g_Ungroup_1() {
    ui_g_Ungroup_1.option.figure_name[0] = 0;
    ui_g_Ungroup_1.option.figure_name[1] = 0;
    ui_g_Ungroup_1.option.figure_name[2] = 3;
    ui_g_Ungroup_1.option.operate_type = 1;

    ui_g_Ungroup_aim->figure_type = 7;
    ui_g_Ungroup_aim->operate_type = 1;
    ui_g_Ungroup_aim->layer = 0;
    ui_g_Ungroup_aim->color = 2;
    ui_g_Ungroup_aim->start_x = 875;
    ui_g_Ungroup_aim->start_y = 460;
    ui_g_Ungroup_aim->width = 3;
    ui_g_Ungroup_aim->font_size = 30;
    ui_g_Ungroup_aim->str_length = 6;
    strcpy(ui_g_Ungroup_aim->string, "      ");


    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_update_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_remove_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
    _ui_init_g_Ungroup_1();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
    _ui_update_g_Ungroup_1();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
    _ui_remove_g_Ungroup_1();
}

