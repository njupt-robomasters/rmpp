//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_30_0;

ui_interface_arc_t *ui_g_30_w1 = (ui_interface_arc_t*)&(ui_g_30_0.data[0]);
ui_interface_arc_t *ui_g_30_w2 = (ui_interface_arc_t*)&(ui_g_30_0.data[1]);
ui_interface_arc_t *ui_g_30_w3 = (ui_interface_arc_t*)&(ui_g_30_0.data[2]);
ui_interface_arc_t *ui_g_30_w4 = (ui_interface_arc_t*)&(ui_g_30_0.data[3]);
ui_interface_rect_t *ui_g_30_s1 = (ui_interface_rect_t*)&(ui_g_30_0.data[4]);
ui_interface_rect_t *ui_g_30_s2 = (ui_interface_rect_t*)&(ui_g_30_0.data[5]);
ui_interface_rect_t *ui_g_30_s3 = (ui_interface_rect_t*)&(ui_g_30_0.data[6]);

void _ui_init_g_30_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30_0.data[i].figure_name[0] = 0;
        ui_g_30_0.data[i].figure_name[1] = 0;
        ui_g_30_0.data[i].figure_name[2] = i + 0;
        ui_g_30_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_30_0.data[i].operate_type = 0;
    }

    ui_g_30_w1->figure_type = 4;
    ui_g_30_w1->operate_type = 1;
    ui_g_30_w1->layer = 3;
    ui_g_30_w1->color = 2;
    ui_g_30_w1->start_x = 290;
    ui_g_30_w1->start_y = 551;
    ui_g_30_w1->width = 2;
    ui_g_30_w1->start_angle = 90;
    ui_g_30_w1->end_angle = 217;
    ui_g_30_w1->rx = 30;
    ui_g_30_w1->ry = 25;

    ui_g_30_w2->figure_type = 4;
    ui_g_30_w2->operate_type = 1;
    ui_g_30_w2->layer = 3;
    ui_g_30_w2->color = 2;
    ui_g_30_w2->start_x = 142;
    ui_g_30_w2->start_y = 551;
    ui_g_30_w2->width = 2;
    ui_g_30_w2->start_angle = 90;
    ui_g_30_w2->end_angle = 217;
    ui_g_30_w2->rx = 30;
    ui_g_30_w2->ry = 25;

    ui_g_30_w3->figure_type = 4;
    ui_g_30_w3->operate_type = 1;
    ui_g_30_w3->layer = 3;
    ui_g_30_w3->color = 2;
    ui_g_30_w3->start_x = 100;
    ui_g_30_w3->start_y = 546;
    ui_g_30_w3->width = 2;
    ui_g_30_w3->start_angle = 90;
    ui_g_30_w3->end_angle = 270;
    ui_g_30_w3->rx = 30;
    ui_g_30_w3->ry = 25;

    ui_g_30_w4->figure_type = 4;
    ui_g_30_w4->operate_type = 1;
    ui_g_30_w4->layer = 3;
    ui_g_30_w4->color = 2;
    ui_g_30_w4->start_x = 250;
    ui_g_30_w4->start_y = 546;
    ui_g_30_w4->width = 2;
    ui_g_30_w4->start_angle = 90;
    ui_g_30_w4->end_angle = 270;
    ui_g_30_w4->rx = 30;
    ui_g_30_w4->ry = 25;

    ui_g_30_s1->figure_type = 1;
    ui_g_30_s1->operate_type = 1;
    ui_g_30_s1->layer = 3;
    ui_g_30_s1->color = 2;
    ui_g_30_s1->start_x = 280;
    ui_g_30_s1->start_y = 551;
    ui_g_30_s1->width = 2;
    ui_g_30_s1->end_x = 320;
    ui_g_30_s1->end_y = 571;

    ui_g_30_s2->figure_type = 1;
    ui_g_30_s2->operate_type = 1;
    ui_g_30_s2->layer = 3;
    ui_g_30_s2->color = 2;
    ui_g_30_s2->start_x = 132;
    ui_g_30_s2->start_y = 551;
    ui_g_30_s2->width = 2;
    ui_g_30_s2->end_x = 172;
    ui_g_30_s2->end_y = 571;

    ui_g_30_s3->figure_type = 1;
    ui_g_30_s3->operate_type = 1;
    ui_g_30_s3->layer = 3;
    ui_g_30_s3->color = 2;
    ui_g_30_s3->start_x = 70;
    ui_g_30_s3->start_y = 546;
    ui_g_30_s3->width = 2;
    ui_g_30_s3->end_x = 130;
    ui_g_30_s3->end_y = 571;


    ui_proc_7_frame(&ui_g_30_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30_0, sizeof(ui_g_30_0));
}

void _ui_update_g_30_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_30_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30_0, sizeof(ui_g_30_0));
}

void _ui_remove_g_30_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_30_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_30_0);
    SEND_MESSAGE((uint8_t *) &ui_g_30_0, sizeof(ui_g_30_0));
}


void ui_init_g_30() {
    _ui_init_g_30_0();
}

void ui_update_g_30() {
    _ui_update_g_30_0();
}

void ui_remove_g_30() {
    _ui_remove_g_30_0();
}

ui_7_frame_t ui_g_31_0;

ui_interface_rect_t *ui_g_31_s4 = (ui_interface_rect_t*)&(ui_g_31_0.data[0]);
ui_interface_rect_t *ui_g_31_cap = (ui_interface_rect_t*)&(ui_g_31_0.data[1]);
ui_interface_rect_t *ui_g_31_yaw1 = (ui_interface_rect_t*)&(ui_g_31_0.data[2]);
ui_interface_rect_t *ui_g_31_yaw2 = (ui_interface_rect_t*)&(ui_g_31_0.data[3]);
ui_interface_rect_t *ui_g_31_pitch = (ui_interface_rect_t*)&(ui_g_31_0.data[4]);
ui_interface_rect_t *ui_g_31_rub_left1 = (ui_interface_rect_t*)&(ui_g_31_0.data[5]);
ui_interface_rect_t *ui_g_31_rub_right1 = (ui_interface_rect_t*)&(ui_g_31_0.data[6]);

void _ui_init_g_31_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_31_0.data[i].figure_name[0] = 0;
        ui_g_31_0.data[i].figure_name[1] = 1;
        ui_g_31_0.data[i].figure_name[2] = i + 0;
        ui_g_31_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_31_0.data[i].operate_type = 0;
    }

    ui_g_31_s4->figure_type = 1;
    ui_g_31_s4->operate_type = 1;
    ui_g_31_s4->layer = 3;
    ui_g_31_s4->color = 2;
    ui_g_31_s4->start_x = 220;
    ui_g_31_s4->start_y = 546;
    ui_g_31_s4->width = 2;
    ui_g_31_s4->end_x = 280;
    ui_g_31_s4->end_y = 571;

    ui_g_31_cap->figure_type = 1;
    ui_g_31_cap->operate_type = 1;
    ui_g_31_cap->layer = 3;
    ui_g_31_cap->color = 2;
    ui_g_31_cap->start_x = 163;
    ui_g_31_cap->start_y = 588;
    ui_g_31_cap->width = 2;
    ui_g_31_cap->end_x = 223;
    ui_g_31_cap->end_y = 608;

    ui_g_31_yaw1->figure_type = 1;
    ui_g_31_yaw1->operate_type = 1;
    ui_g_31_yaw1->layer = 3;
    ui_g_31_yaw1->color = 2;
    ui_g_31_yaw1->start_x = 56;
    ui_g_31_yaw1->start_y = 573;
    ui_g_31_yaw1->width = 2;
    ui_g_31_yaw1->end_x = 331;
    ui_g_31_yaw1->end_y = 623;

    ui_g_31_yaw2->figure_type = 1;
    ui_g_31_yaw2->operate_type = 1;
    ui_g_31_yaw2->layer = 3;
    ui_g_31_yaw2->color = 2;
    ui_g_31_yaw2->start_x = 116;
    ui_g_31_yaw2->start_y = 639;
    ui_g_31_yaw2->width = 2;
    ui_g_31_yaw2->end_x = 241;
    ui_g_31_yaw2->end_y = 689;

    ui_g_31_pitch->figure_type = 1;
    ui_g_31_pitch->operate_type = 1;
    ui_g_31_pitch->layer = 3;
    ui_g_31_pitch->color = 2;
    ui_g_31_pitch->start_x = 243;
    ui_g_31_pitch->start_y = 648;
    ui_g_31_pitch->width = 2;
    ui_g_31_pitch->end_x = 323;
    ui_g_31_pitch->end_y = 678;

    ui_g_31_rub_left1->figure_type = 1;
    ui_g_31_rub_left1->operate_type = 1;
    ui_g_31_rub_left1->layer = 3;
    ui_g_31_rub_left1->color = 2;
    ui_g_31_rub_left1->start_x = 257;
    ui_g_31_rub_left1->start_y = 664;
    ui_g_31_rub_left1->width = 2;
    ui_g_31_rub_left1->end_x = 269;
    ui_g_31_rub_left1->end_y = 676;

    ui_g_31_rub_right1->figure_type = 1;
    ui_g_31_rub_right1->operate_type = 1;
    ui_g_31_rub_right1->layer = 3;
    ui_g_31_rub_right1->color = 2;
    ui_g_31_rub_right1->start_x = 257;
    ui_g_31_rub_right1->start_y = 650;
    ui_g_31_rub_right1->width = 2;
    ui_g_31_rub_right1->end_x = 269;
    ui_g_31_rub_right1->end_y = 662;


    ui_proc_7_frame(&ui_g_31_0);
    SEND_MESSAGE((uint8_t *) &ui_g_31_0, sizeof(ui_g_31_0));
}

void _ui_update_g_31_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_31_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_31_0);
    SEND_MESSAGE((uint8_t *) &ui_g_31_0, sizeof(ui_g_31_0));
}

void _ui_remove_g_31_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_31_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_31_0);
    SEND_MESSAGE((uint8_t *) &ui_g_31_0, sizeof(ui_g_31_0));
}


void ui_init_g_31() {
    _ui_init_g_31_0();
}

void ui_update_g_31() {
    _ui_update_g_31_0();
}

void ui_remove_g_31() {
    _ui_remove_g_31_0();
}

ui_5_frame_t ui_g_32_0;

ui_interface_rect_t *ui_g_32_rub_left2 = (ui_interface_rect_t*)&(ui_g_32_0.data[0]);
ui_interface_rect_t *ui_g_32_rub_right2 = (ui_interface_rect_t*)&(ui_g_32_0.data[1]);
ui_interface_rect_t *ui_g_32_shoot = (ui_interface_rect_t*)&(ui_g_32_0.data[2]);
ui_interface_rect_t *ui_g_32_aim = (ui_interface_rect_t*)&(ui_g_32_0.data[3]);

void _ui_init_g_32_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_32_0.data[i].figure_name[0] = 0;
        ui_g_32_0.data[i].figure_name[1] = 2;
        ui_g_32_0.data[i].figure_name[2] = i + 0;
        ui_g_32_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_32_0.data[i].operate_type = 0;
    }

    ui_g_32_rub_left2->figure_type = 1;
    ui_g_32_rub_left2->operate_type = 1;
    ui_g_32_rub_left2->layer = 3;
    ui_g_32_rub_left2->color = 2;
    ui_g_32_rub_left2->start_x = 271;
    ui_g_32_rub_left2->start_y = 664;
    ui_g_32_rub_left2->width = 2;
    ui_g_32_rub_left2->end_x = 283;
    ui_g_32_rub_left2->end_y = 676;

    ui_g_32_rub_right2->figure_type = 1;
    ui_g_32_rub_right2->operate_type = 1;
    ui_g_32_rub_right2->layer = 3;
    ui_g_32_rub_right2->color = 2;
    ui_g_32_rub_right2->start_x = 271;
    ui_g_32_rub_right2->start_y = 650;
    ui_g_32_rub_right2->width = 2;
    ui_g_32_rub_right2->end_x = 283;
    ui_g_32_rub_right2->end_y = 662;

    ui_g_32_shoot->figure_type = 1;
    ui_g_32_shoot->operate_type = 1;
    ui_g_32_shoot->layer = 3;
    ui_g_32_shoot->color = 2;
    ui_g_32_shoot->start_x = 156;
    ui_g_32_shoot->start_y = 625;
    ui_g_32_shoot->width = 2;
    ui_g_32_shoot->end_x = 231;
    ui_g_32_shoot->end_y = 637;

    ui_g_32_aim->figure_type = 1;
    ui_g_32_aim->operate_type = 1;
    ui_g_32_aim->layer = 3;
    ui_g_32_aim->color = 2;
    ui_g_32_aim->start_x = 298;
    ui_g_32_aim->start_y = 680;
    ui_g_32_aim->width = 2;
    ui_g_32_aim->end_x = 323;
    ui_g_32_aim->end_y = 692;


    ui_proc_5_frame(&ui_g_32_0);
    SEND_MESSAGE((uint8_t *) &ui_g_32_0, sizeof(ui_g_32_0));
}

void _ui_update_g_32_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_32_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_32_0);
    SEND_MESSAGE((uint8_t *) &ui_g_32_0, sizeof(ui_g_32_0));
}

void _ui_remove_g_32_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_32_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_32_0);
    SEND_MESSAGE((uint8_t *) &ui_g_32_0, sizeof(ui_g_32_0));
}


void ui_init_g_32() {
    _ui_init_g_32_0();
}

void ui_update_g_32() {
    _ui_update_g_32_0();
}

void ui_remove_g_32() {
    _ui_remove_g_32_0();
}

