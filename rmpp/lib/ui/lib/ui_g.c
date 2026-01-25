//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_Ungroup_0;

ui_interface_round_t *ui_g_Ungroup_yaw_round = (ui_interface_round_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_number_t *ui_g_Ungroup_bullet_speed_2 = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_line_t *ui_g_Ungroup_yaw = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[2]);
ui_interface_arc_t *ui_g_Ungroup_hit = (ui_interface_arc_t*)&(ui_g_Ungroup_0.data[3]);
ui_interface_number_t *ui_g_Ungroup_enemy_hp = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[4]);
ui_interface_number_t *ui_g_Ungroup_shoot_current = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[5]);
ui_interface_number_t *ui_g_Ungroup_bullet_speed_1 = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[6]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_yaw_round->figure_type = 2;
    ui_g_Ungroup_yaw_round->operate_type = 1;
    ui_g_Ungroup_yaw_round->layer = 0;
    ui_g_Ungroup_yaw_round->color = 2;
    ui_g_Ungroup_yaw_round->start_x = 960;
    ui_g_Ungroup_yaw_round->start_y = 250;
    ui_g_Ungroup_yaw_round->width = 3;
    ui_g_Ungroup_yaw_round->r = 50;

    ui_g_Ungroup_bullet_speed_2->figure_type = 5;
    ui_g_Ungroup_bullet_speed_2->operate_type = 1;
    ui_g_Ungroup_bullet_speed_2->layer = 0;
    ui_g_Ungroup_bullet_speed_2->color = 2;
    ui_g_Ungroup_bullet_speed_2->start_x = 1610;
    ui_g_Ungroup_bullet_speed_2->start_y = 585;
    ui_g_Ungroup_bullet_speed_2->width = 2;
    ui_g_Ungroup_bullet_speed_2->font_size = 20;
    ui_g_Ungroup_bullet_speed_2->number = 12345;

    ui_g_Ungroup_yaw->figure_type = 0;
    ui_g_Ungroup_yaw->operate_type = 1;
    ui_g_Ungroup_yaw->layer = 0;
    ui_g_Ungroup_yaw->color = 2;
    ui_g_Ungroup_yaw->start_x = 960;
    ui_g_Ungroup_yaw->start_y = 250;
    ui_g_Ungroup_yaw->width = 3;
    ui_g_Ungroup_yaw->end_x = 960;
    ui_g_Ungroup_yaw->end_y = 300;

    ui_g_Ungroup_hit->figure_type = 4;
    ui_g_Ungroup_hit->operate_type = 1;
    ui_g_Ungroup_hit->layer = 0;
    ui_g_Ungroup_hit->color = 1;
    ui_g_Ungroup_hit->start_x = 960;
    ui_g_Ungroup_hit->start_y = 540;
    ui_g_Ungroup_hit->width = 20;
    ui_g_Ungroup_hit->start_angle = 342;
    ui_g_Ungroup_hit->end_angle = 18;
    ui_g_Ungroup_hit->rx = 360;
    ui_g_Ungroup_hit->ry = 360;

    ui_g_Ungroup_enemy_hp->figure_type = 6;
    ui_g_Ungroup_enemy_hp->operate_type = 1;
    ui_g_Ungroup_enemy_hp->layer = 0;
    ui_g_Ungroup_enemy_hp->color = 2;
    ui_g_Ungroup_enemy_hp->start_x = 915;
    ui_g_Ungroup_enemy_hp->start_y = 400;
    ui_g_Ungroup_enemy_hp->width = 3;
    ui_g_Ungroup_enemy_hp->font_size = 30;
    ui_g_Ungroup_enemy_hp->number = 200;

    ui_g_Ungroup_shoot_current->figure_type = 5;
    ui_g_Ungroup_shoot_current->operate_type = 1;
    ui_g_Ungroup_shoot_current->layer = 0;
    ui_g_Ungroup_shoot_current->color = 2;
    ui_g_Ungroup_shoot_current->start_x = 1475;
    ui_g_Ungroup_shoot_current->start_y = 530;
    ui_g_Ungroup_shoot_current->width = 2;
    ui_g_Ungroup_shoot_current->font_size = 20;
    ui_g_Ungroup_shoot_current->number = 10000;

    ui_g_Ungroup_bullet_speed_1->figure_type = 5;
    ui_g_Ungroup_bullet_speed_1->operate_type = 1;
    ui_g_Ungroup_bullet_speed_1->layer = 0;
    ui_g_Ungroup_bullet_speed_1->color = 2;
    ui_g_Ungroup_bullet_speed_1->start_x = 1475;
    ui_g_Ungroup_bullet_speed_1->start_y = 585;
    ui_g_Ungroup_bullet_speed_1->width = 2;
    ui_g_Ungroup_bullet_speed_1->font_size = 20;
    ui_g_Ungroup_bullet_speed_1->number = 12345;


    ui_proc_7_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

ui_string_frame_t ui_g_Ungroup_1;
ui_interface_string_t* ui_g_Ungroup_aim = &(ui_g_Ungroup_1.option);

void _ui_init_g_Ungroup_1() {
    ui_g_Ungroup_1.option.figure_name[0] = 0;
    ui_g_Ungroup_1.option.figure_name[1] = 0;
    ui_g_Ungroup_1.option.figure_name[2] = 7;
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
    strcpy(ui_g_Ungroup_aim->string, "ONLINE");


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

