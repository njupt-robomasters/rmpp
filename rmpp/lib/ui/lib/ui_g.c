//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_00_0;

ui_interface_line_t *ui_g_00_line0 = (ui_interface_line_t*)&(ui_g_00_0.data[0]);
ui_interface_line_t *ui_g_00_line1 = (ui_interface_line_t*)&(ui_g_00_0.data[1]);
ui_interface_line_t *ui_g_00_line2 = (ui_interface_line_t*)&(ui_g_00_0.data[2]);
ui_interface_line_t *ui_g_00_line3 = (ui_interface_line_t*)&(ui_g_00_0.data[3]);

void _ui_init_g_00_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_00_0.data[i].figure_name[0] = 0;
        ui_g_00_0.data[i].figure_name[1] = 0;
        ui_g_00_0.data[i].figure_name[2] = i + 0;
        ui_g_00_0.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_g_00_0.data[i].operate_type = 0;
    }

    ui_g_00_line0->figure_type = 0;
    ui_g_00_line0->operate_type = 1;
    ui_g_00_line0->layer = 0;
    ui_g_00_line0->color = 8;
    ui_g_00_line0->start_x = 960;
    ui_g_00_line0->start_y = 660;
    ui_g_00_line0->width = 2;
    ui_g_00_line0->end_x = 960;
    ui_g_00_line0->end_y = 890;

    ui_g_00_line1->figure_type = 0;
    ui_g_00_line1->operate_type = 1;
    ui_g_00_line1->layer = 0;
    ui_g_00_line1->color = 8;
    ui_g_00_line1->start_x = 840;
    ui_g_00_line1->start_y = 540;
    ui_g_00_line1->width = 2;
    ui_g_00_line1->end_x = 610;
    ui_g_00_line1->end_y = 540;

    ui_g_00_line2->figure_type = 0;
    ui_g_00_line2->operate_type = 1;
    ui_g_00_line2->layer = 0;
    ui_g_00_line2->color = 8;
    ui_g_00_line2->start_x = 960;
    ui_g_00_line2->start_y = 420;
    ui_g_00_line2->width = 2;
    ui_g_00_line2->end_x = 960;
    ui_g_00_line2->end_y = 190;

    ui_g_00_line3->figure_type = 0;
    ui_g_00_line3->operate_type = 1;
    ui_g_00_line3->layer = 0;
    ui_g_00_line3->color = 8;
    ui_g_00_line3->start_x = 1080;
    ui_g_00_line3->start_y = 540;
    ui_g_00_line3->width = 2;
    ui_g_00_line3->end_x = 1310;
    ui_g_00_line3->end_y = 540;


    ui_proc_5_frame(&ui_g_00_0);
    SEND_MESSAGE((uint8_t *) &ui_g_00_0, sizeof(ui_g_00_0));
}

void _ui_update_g_00_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_00_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_00_0);
    SEND_MESSAGE((uint8_t *) &ui_g_00_0, sizeof(ui_g_00_0));
}

void _ui_remove_g_00_0() {
    for (int i = 0; i < 4; i++) {
        ui_g_00_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_00_0);
    SEND_MESSAGE((uint8_t *) &ui_g_00_0, sizeof(ui_g_00_0));
}


void ui_init_g_00() {
    _ui_init_g_00_0();
}

void ui_update_g_00() {
    _ui_update_g_00_0();
}

void ui_remove_g_00() {
    _ui_remove_g_00_0();
}

ui_7_frame_t ui_g_01_0;

ui_interface_arc_t *ui_g_01_limit0 = (ui_interface_arc_t*)&(ui_g_01_0.data[0]);
ui_interface_arc_t *ui_g_01_limit1 = (ui_interface_arc_t*)&(ui_g_01_0.data[1]);
ui_interface_arc_t *ui_g_01_limit2 = (ui_interface_arc_t*)&(ui_g_01_0.data[2]);
ui_interface_arc_t *ui_g_01_limit3 = (ui_interface_arc_t*)&(ui_g_01_0.data[3]);
ui_interface_arc_t *ui_g_01_limit4 = (ui_interface_arc_t*)&(ui_g_01_0.data[4]);
ui_interface_arc_t *ui_g_01_limit5 = (ui_interface_arc_t*)&(ui_g_01_0.data[5]);

void _ui_init_g_01_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_01_0.data[i].figure_name[0] = 0;
        ui_g_01_0.data[i].figure_name[1] = 1;
        ui_g_01_0.data[i].figure_name[2] = i + 0;
        ui_g_01_0.data[i].operate_type = 1;
    }
    for (int i = 6; i < 7; i++) {
        ui_g_01_0.data[i].operate_type = 0;
    }

    ui_g_01_limit0->figure_type = 4;
    ui_g_01_limit0->operate_type = 1;
    ui_g_01_limit0->layer = 0;
    ui_g_01_limit0->color = 8;
    ui_g_01_limit0->start_x = 960;
    ui_g_01_limit0->start_y = 540;
    ui_g_01_limit0->width = 18;
    ui_g_01_limit0->start_angle = 315;
    ui_g_01_limit0->end_angle = 317;
    ui_g_01_limit0->rx = 387;
    ui_g_01_limit0->ry = 387;

    ui_g_01_limit1->figure_type = 4;
    ui_g_01_limit1->operate_type = 1;
    ui_g_01_limit1->layer = 0;
    ui_g_01_limit1->color = 8;
    ui_g_01_limit1->start_x = 960;
    ui_g_01_limit1->start_y = 540;
    ui_g_01_limit1->width = 18;
    ui_g_01_limit1->start_angle = 267;
    ui_g_01_limit1->end_angle = 273;
    ui_g_01_limit1->rx = 387;
    ui_g_01_limit1->ry = 387;

    ui_g_01_limit2->figure_type = 4;
    ui_g_01_limit2->operate_type = 1;
    ui_g_01_limit2->layer = 0;
    ui_g_01_limit2->color = 8;
    ui_g_01_limit2->start_x = 960;
    ui_g_01_limit2->start_y = 540;
    ui_g_01_limit2->width = 18;
    ui_g_01_limit2->start_angle = 223;
    ui_g_01_limit2->end_angle = 225;
    ui_g_01_limit2->rx = 387;
    ui_g_01_limit2->ry = 387;

    ui_g_01_limit3->figure_type = 4;
    ui_g_01_limit3->operate_type = 1;
    ui_g_01_limit3->layer = 0;
    ui_g_01_limit3->color = 8;
    ui_g_01_limit3->start_x = 960;
    ui_g_01_limit3->start_y = 540;
    ui_g_01_limit3->width = 18;
    ui_g_01_limit3->start_angle = 135;
    ui_g_01_limit3->end_angle = 137;
    ui_g_01_limit3->rx = 387;
    ui_g_01_limit3->ry = 387;

    ui_g_01_limit4->figure_type = 4;
    ui_g_01_limit4->operate_type = 1;
    ui_g_01_limit4->layer = 0;
    ui_g_01_limit4->color = 8;
    ui_g_01_limit4->start_x = 960;
    ui_g_01_limit4->start_y = 540;
    ui_g_01_limit4->width = 18;
    ui_g_01_limit4->start_angle = 87;
    ui_g_01_limit4->end_angle = 93;
    ui_g_01_limit4->rx = 387;
    ui_g_01_limit4->ry = 387;

    ui_g_01_limit5->figure_type = 4;
    ui_g_01_limit5->operate_type = 1;
    ui_g_01_limit5->layer = 0;
    ui_g_01_limit5->color = 8;
    ui_g_01_limit5->start_x = 960;
    ui_g_01_limit5->start_y = 540;
    ui_g_01_limit5->width = 18;
    ui_g_01_limit5->start_angle = 43;
    ui_g_01_limit5->end_angle = 45;
    ui_g_01_limit5->rx = 387;
    ui_g_01_limit5->ry = 387;


    ui_proc_7_frame(&ui_g_01_0);
    SEND_MESSAGE((uint8_t *) &ui_g_01_0, sizeof(ui_g_01_0));
}

void _ui_update_g_01_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_01_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_01_0);
    SEND_MESSAGE((uint8_t *) &ui_g_01_0, sizeof(ui_g_01_0));
}

void _ui_remove_g_01_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_01_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_01_0);
    SEND_MESSAGE((uint8_t *) &ui_g_01_0, sizeof(ui_g_01_0));
}


void ui_init_g_01() {
    _ui_init_g_01_0();
}

void ui_update_g_01() {
    _ui_update_g_01_0();
}

void ui_remove_g_01() {
    _ui_remove_g_01_0();
}


ui_string_frame_t ui_g_02_0;
ui_interface_string_t* ui_g_02_text0 = &(ui_g_02_0.option);

void _ui_init_g_02_0() {
    ui_g_02_0.option.figure_name[0] = 0;
    ui_g_02_0.option.figure_name[1] = 2;
    ui_g_02_0.option.figure_name[2] = 0;
    ui_g_02_0.option.operate_type = 1;

    ui_g_02_text0->figure_type = 7;
    ui_g_02_text0->operate_type = 1;
    ui_g_02_text0->layer = 0;
    ui_g_02_text0->color = 8;
    ui_g_02_text0->start_x = 660;
    ui_g_02_text0->start_y = 860;
    ui_g_02_text0->width = 2;
    ui_g_02_text0->font_size = 20;
    ui_g_02_text0->str_length = 3;
    strcpy(ui_g_02_text0->string, "CAP");


    ui_proc_string_frame(&ui_g_02_0);
    SEND_MESSAGE((uint8_t *) &ui_g_02_0, sizeof(ui_g_02_0));
}

void _ui_update_g_02_0() {
    ui_g_02_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_02_0);
    SEND_MESSAGE((uint8_t *) &ui_g_02_0, sizeof(ui_g_02_0));
}

void _ui_remove_g_02_0() {
    ui_g_02_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_02_0);
    SEND_MESSAGE((uint8_t *) &ui_g_02_0, sizeof(ui_g_02_0));
}

void ui_init_g_02() {
    _ui_init_g_02_0();
}

void ui_update_g_02() {
    _ui_update_g_02_0();
}

void ui_remove_g_02() {
    _ui_remove_g_02_0();
}


ui_string_frame_t ui_g_03_0;
ui_interface_string_t* ui_g_03_text1 = &(ui_g_03_0.option);

void _ui_init_g_03_0() {
    ui_g_03_0.option.figure_name[0] = 0;
    ui_g_03_0.option.figure_name[1] = 3;
    ui_g_03_0.option.figure_name[2] = 0;
    ui_g_03_0.option.operate_type = 1;

    ui_g_03_text1->figure_type = 7;
    ui_g_03_text1->operate_type = 1;
    ui_g_03_text1->layer = 0;
    ui_g_03_text1->color = 8;
    ui_g_03_text1->start_x = 660;
    ui_g_03_text1->start_y = 240;
    ui_g_03_text1->width = 2;
    ui_g_03_text1->font_size = 20;
    ui_g_03_text1->str_length = 3;
    strcpy(ui_g_03_text1->string, "RPM");


    ui_proc_string_frame(&ui_g_03_0);
    SEND_MESSAGE((uint8_t *) &ui_g_03_0, sizeof(ui_g_03_0));
}

void _ui_update_g_03_0() {
    ui_g_03_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_03_0);
    SEND_MESSAGE((uint8_t *) &ui_g_03_0, sizeof(ui_g_03_0));
}

void _ui_remove_g_03_0() {
    ui_g_03_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_03_0);
    SEND_MESSAGE((uint8_t *) &ui_g_03_0, sizeof(ui_g_03_0));
}

void ui_init_g_03() {
    _ui_init_g_03_0();
}

void ui_update_g_03() {
    _ui_update_g_03_0();
}

void ui_remove_g_03() {
    _ui_remove_g_03_0();
}


ui_string_frame_t ui_g_04_0;
ui_interface_string_t* ui_g_04_text2 = &(ui_g_04_0.option);

void _ui_init_g_04_0() {
    ui_g_04_0.option.figure_name[0] = 0;
    ui_g_04_0.option.figure_name[1] = 4;
    ui_g_04_0.option.figure_name[2] = 0;
    ui_g_04_0.option.operate_type = 1;

    ui_g_04_text2->figure_type = 7;
    ui_g_04_text2->operate_type = 1;
    ui_g_04_text2->layer = 0;
    ui_g_04_text2->color = 8;
    ui_g_04_text2->start_x = 1200;
    ui_g_04_text2->start_y = 240;
    ui_g_04_text2->width = 2;
    ui_g_04_text2->font_size = 20;
    ui_g_04_text2->str_length = 4;
    strcpy(ui_g_04_text2->string, "FREQ");


    ui_proc_string_frame(&ui_g_04_0);
    SEND_MESSAGE((uint8_t *) &ui_g_04_0, sizeof(ui_g_04_0));
}

void _ui_update_g_04_0() {
    ui_g_04_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_04_0);
    SEND_MESSAGE((uint8_t *) &ui_g_04_0, sizeof(ui_g_04_0));
}

void _ui_remove_g_04_0() {
    ui_g_04_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_04_0);
    SEND_MESSAGE((uint8_t *) &ui_g_04_0, sizeof(ui_g_04_0));
}

void ui_init_g_04() {
    _ui_init_g_04_0();
}

void ui_update_g_04() {
    _ui_update_g_04_0();
}

void ui_remove_g_04() {
    _ui_remove_g_04_0();
}


ui_string_frame_t ui_g_05_0;
ui_interface_string_t* ui_g_05_text3 = &(ui_g_05_0.option);

void _ui_init_g_05_0() {
    ui_g_05_0.option.figure_name[0] = 0;
    ui_g_05_0.option.figure_name[1] = 5;
    ui_g_05_0.option.figure_name[2] = 0;
    ui_g_05_0.option.operate_type = 1;

    ui_g_05_text3->figure_type = 7;
    ui_g_05_text3->operate_type = 1;
    ui_g_05_text3->layer = 0;
    ui_g_05_text3->color = 8;
    ui_g_05_text3->start_x = 1200;
    ui_g_05_text3->start_y = 860;
    ui_g_05_text3->width = 2;
    ui_g_05_text3->font_size = 20;
    ui_g_05_text3->str_length = 5;
    strcpy(ui_g_05_text3->string, "SHOOT");


    ui_proc_string_frame(&ui_g_05_0);
    SEND_MESSAGE((uint8_t *) &ui_g_05_0, sizeof(ui_g_05_0));
}

void _ui_update_g_05_0() {
    ui_g_05_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_05_0);
    SEND_MESSAGE((uint8_t *) &ui_g_05_0, sizeof(ui_g_05_0));
}

void _ui_remove_g_05_0() {
    ui_g_05_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_05_0);
    SEND_MESSAGE((uint8_t *) &ui_g_05_0, sizeof(ui_g_05_0));
}

void ui_init_g_05() {
    _ui_init_g_05_0();
}

void ui_update_g_05() {
    _ui_update_g_05_0();
}

void ui_remove_g_05() {
    _ui_remove_g_05_0();
}

ui_7_frame_t ui_g_1_0;

ui_interface_arc_t *ui_g_1_dir = (ui_interface_arc_t*)&(ui_g_1_0.data[0]);
ui_interface_arc_t *ui_g_1_hurt = (ui_interface_arc_t*)&(ui_g_1_0.data[1]);
ui_interface_arc_t *ui_g_1_bar0 = (ui_interface_arc_t*)&(ui_g_1_0.data[2]);
ui_interface_arc_t *ui_g_1_bar1 = (ui_interface_arc_t*)&(ui_g_1_0.data[3]);
ui_interface_arc_t *ui_g_1_bar2 = (ui_interface_arc_t*)&(ui_g_1_0.data[4]);
ui_interface_arc_t *ui_g_1_bar3 = (ui_interface_arc_t*)&(ui_g_1_0.data[5]);

void _ui_init_g_1_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_1_0.data[i].figure_name[0] = 0;
        ui_g_1_0.data[i].figure_name[1] = 6;
        ui_g_1_0.data[i].figure_name[2] = i + 0;
        ui_g_1_0.data[i].operate_type = 1;
    }
    for (int i = 6; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 0;
    }

    ui_g_1_dir->figure_type = 4;
    ui_g_1_dir->operate_type = 1;
    ui_g_1_dir->layer = 1;
    ui_g_1_dir->color = 2;
    ui_g_1_dir->start_x = 960;
    ui_g_1_dir->start_y = 540;
    ui_g_1_dir->width = 10;
    ui_g_1_dir->start_angle = 0;
    ui_g_1_dir->end_angle = 360;
    ui_g_1_dir->rx = 85;
    ui_g_1_dir->ry = 85;

    ui_g_1_hurt->figure_type = 4;
    ui_g_1_hurt->operate_type = 1;
    ui_g_1_hurt->layer = 1;
    ui_g_1_hurt->color = 5;
    ui_g_1_hurt->start_x = 960;
    ui_g_1_hurt->start_y = 540;
    ui_g_1_hurt->width = 10;
    ui_g_1_hurt->start_angle = 0;
    ui_g_1_hurt->end_angle = 360;
    ui_g_1_hurt->rx = 85;
    ui_g_1_hurt->ry = 85;

    ui_g_1_bar0->figure_type = 4;
    ui_g_1_bar0->operate_type = 1;
    ui_g_1_bar0->layer = 1;
    ui_g_1_bar0->color = 2;
    ui_g_1_bar0->start_x = 960;
    ui_g_1_bar0->start_y = 540;
    ui_g_1_bar0->width = 14;
    ui_g_1_bar0->start_angle = 275;
    ui_g_1_bar0->end_angle = 313;
    ui_g_1_bar0->rx = 387;
    ui_g_1_bar0->ry = 387;

    ui_g_1_bar1->figure_type = 4;
    ui_g_1_bar1->operate_type = 1;
    ui_g_1_bar1->layer = 1;
    ui_g_1_bar1->color = 2;
    ui_g_1_bar1->start_x = 960;
    ui_g_1_bar1->start_y = 540;
    ui_g_1_bar1->width = 14;
    ui_g_1_bar1->start_angle = 227;
    ui_g_1_bar1->end_angle = 265;
    ui_g_1_bar1->rx = 387;
    ui_g_1_bar1->ry = 387;

    ui_g_1_bar2->figure_type = 4;
    ui_g_1_bar2->operate_type = 1;
    ui_g_1_bar2->layer = 1;
    ui_g_1_bar2->color = 2;
    ui_g_1_bar2->start_x = 960;
    ui_g_1_bar2->start_y = 540;
    ui_g_1_bar2->width = 14;
    ui_g_1_bar2->start_angle = 95;
    ui_g_1_bar2->end_angle = 133;
    ui_g_1_bar2->rx = 387;
    ui_g_1_bar2->ry = 387;

    ui_g_1_bar3->figure_type = 4;
    ui_g_1_bar3->operate_type = 1;
    ui_g_1_bar3->layer = 1;
    ui_g_1_bar3->color = 2;
    ui_g_1_bar3->start_x = 960;
    ui_g_1_bar3->start_y = 540;
    ui_g_1_bar3->width = 14;
    ui_g_1_bar3->start_angle = 47;
    ui_g_1_bar3->end_angle = 85;
    ui_g_1_bar3->rx = 387;
    ui_g_1_bar3->ry = 387;


    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_update_g_1_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_1_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_remove_g_1_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_1_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}


void ui_init_g_1() {
    _ui_init_g_1_0();
}

void ui_update_g_1() {
    _ui_update_g_1_0();
}

void ui_remove_g_1() {
    _ui_remove_g_1_0();
}

ui_7_frame_t ui_g_2_0;

ui_interface_number_t *ui_g_2_number0 = (ui_interface_number_t*)&(ui_g_2_0.data[0]);
ui_interface_number_t *ui_g_2_number1 = (ui_interface_number_t*)&(ui_g_2_0.data[1]);
ui_interface_number_t *ui_g_2_number2 = (ui_interface_number_t*)&(ui_g_2_0.data[2]);
ui_interface_number_t *ui_g_2_number3 = (ui_interface_number_t*)&(ui_g_2_0.data[3]);
ui_interface_number_t *ui_g_2_yaw = (ui_interface_number_t*)&(ui_g_2_0.data[4]);
ui_interface_number_t *ui_g_2_pitch = (ui_interface_number_t*)&(ui_g_2_0.data[5]);

void _ui_init_g_2_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_2_0.data[i].figure_name[0] = 0;
        ui_g_2_0.data[i].figure_name[1] = 7;
        ui_g_2_0.data[i].figure_name[2] = i + 0;
        ui_g_2_0.data[i].operate_type = 1;
    }
    for (int i = 6; i < 7; i++) {
        ui_g_2_0.data[i].operate_type = 0;
    }

    ui_g_2_number0->figure_type = 6;
    ui_g_2_number0->operate_type = 1;
    ui_g_2_number0->layer = 2;
    ui_g_2_number0->color = 8;
    ui_g_2_number0->start_x = 615;
    ui_g_2_number0->start_y = 575;
    ui_g_2_number0->width = 2;
    ui_g_2_number0->font_size = 15;
    ui_g_2_number0->number = 12345;

    ui_g_2_number1->figure_type = 6;
    ui_g_2_number1->operate_type = 1;
    ui_g_2_number1->layer = 2;
    ui_g_2_number1->color = 8;
    ui_g_2_number1->start_x = 615;
    ui_g_2_number1->start_y = 550;
    ui_g_2_number1->width = 2;
    ui_g_2_number1->font_size = 15;
    ui_g_2_number1->number = 12345;

    ui_g_2_number2->figure_type = 6;
    ui_g_2_number2->operate_type = 1;
    ui_g_2_number2->layer = 2;
    ui_g_2_number2->color = 8;
    ui_g_2_number2->start_x = 1250;
    ui_g_2_number2->start_y = 550;
    ui_g_2_number2->width = 2;
    ui_g_2_number2->font_size = 15;
    ui_g_2_number2->number = 12345;

    ui_g_2_number3->figure_type = 6;
    ui_g_2_number3->operate_type = 1;
    ui_g_2_number3->layer = 2;
    ui_g_2_number3->color = 8;
    ui_g_2_number3->start_x = 1250;
    ui_g_2_number3->start_y = 575;
    ui_g_2_number3->width = 2;
    ui_g_2_number3->font_size = 15;
    ui_g_2_number3->number = 12345;

    ui_g_2_yaw->figure_type = 6;
    ui_g_2_yaw->operate_type = 1;
    ui_g_2_yaw->layer = 2;
    ui_g_2_yaw->color = 8;
    ui_g_2_yaw->start_x = 783;
    ui_g_2_yaw->start_y = 565;
    ui_g_2_yaw->width = 2;
    ui_g_2_yaw->font_size = 15;
    ui_g_2_yaw->number = 12345;

    ui_g_2_pitch->figure_type = 6;
    ui_g_2_pitch->operate_type = 1;
    ui_g_2_pitch->layer = 2;
    ui_g_2_pitch->color = 8;
    ui_g_2_pitch->start_x = 965;
    ui_g_2_pitch->start_y = 688;
    ui_g_2_pitch->width = 2;
    ui_g_2_pitch->font_size = 15;
    ui_g_2_pitch->number = 12345;


    ui_proc_7_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_update_g_2_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_2_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}

void _ui_remove_g_2_0() {
    for (int i = 0; i < 6; i++) {
        ui_g_2_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_2_0, sizeof(ui_g_2_0));
}


void ui_init_g_2() {
    _ui_init_g_2_0();
}

void ui_update_g_2() {
    _ui_update_g_2_0();
}

void ui_remove_g_2() {
    _ui_remove_g_2_0();
}


ui_string_frame_t ui_g_3_0;
ui_interface_string_t* ui_g_3_aim_status = &(ui_g_3_0.option);

void _ui_init_g_3_0() {
    ui_g_3_0.option.figure_name[0] = 0;
    ui_g_3_0.option.figure_name[1] = 8;
    ui_g_3_0.option.figure_name[2] = 0;
    ui_g_3_0.option.operate_type = 1;

    ui_g_3_aim_status->figure_type = 7;
    ui_g_3_aim_status->operate_type = 1;
    ui_g_3_aim_status->layer = 3;
    ui_g_3_aim_status->color = 2;
    ui_g_3_aim_status->start_x = 905;
    ui_g_3_aim_status->start_y = 445;
    ui_g_3_aim_status->width = 2;
    ui_g_3_aim_status->font_size = 20;
    ui_g_3_aim_status->str_length = 6;
    strcpy(ui_g_3_aim_status->string, "ONLINE");


    ui_proc_string_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void _ui_update_g_3_0() {
    ui_g_3_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void _ui_remove_g_3_0() {
    ui_g_3_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_3_0, sizeof(ui_g_3_0));
}

void ui_init_g_3() {
    _ui_init_g_3_0();
}

void ui_update_g_3() {
    _ui_update_g_3_0();
}

void ui_remove_g_3() {
    _ui_remove_g_3_0();
}

