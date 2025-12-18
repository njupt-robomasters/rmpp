//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 6
#define TOTAL_STRING 1

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_Ungroup_hit->figure_type = 4;
    ui_g_Ungroup_hit->operate_type = 1;
    ui_g_Ungroup_hit->layer = 0;
    ui_g_Ungroup_hit->color = 1;
    ui_g_Ungroup_hit->start_x = 960;
    ui_g_Ungroup_hit->start_y = 540;
    ui_g_Ungroup_hit->width = 20;
    ui_g_Ungroup_hit->start_angle = 0;
    ui_g_Ungroup_hit->end_angle = 36;
    ui_g_Ungroup_hit->rx = 390;
    ui_g_Ungroup_hit->ry = 390;

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

    ui_g_Ungroup_cap_rect->figure_type = 1;
    ui_g_Ungroup_cap_rect->operate_type = 1;
    ui_g_Ungroup_cap_rect->layer = 0;
    ui_g_Ungroup_cap_rect->color = 2;
    ui_g_Ungroup_cap_rect->start_x = 710;
    ui_g_Ungroup_cap_rect->start_y = 70;
    ui_g_Ungroup_cap_rect->width = 4;
    ui_g_Ungroup_cap_rect->end_x = 1210;
    ui_g_Ungroup_cap_rect->end_y = 120;

    ui_g_Ungroup_cap->figure_type = 1;
    ui_g_Ungroup_cap->operate_type = 1;
    ui_g_Ungroup_cap->layer = 0;
    ui_g_Ungroup_cap->color = 2;
    ui_g_Ungroup_cap->start_x = 712;
    ui_g_Ungroup_cap->start_y = 72;
    ui_g_Ungroup_cap->width = 25;
    ui_g_Ungroup_cap->end_x = 1187;
    ui_g_Ungroup_cap->end_y = 97;

    ui_g_Ungroup_cap_text->figure_type = 6;
    ui_g_Ungroup_cap_text->operate_type = 1;
    ui_g_Ungroup_cap_text->layer = 0;
    ui_g_Ungroup_cap_text->color = 2;
    ui_g_Ungroup_cap_text->start_x = 1225;
    ui_g_Ungroup_cap_text->start_y = 130;
    ui_g_Ungroup_cap_text->width = 3;
    ui_g_Ungroup_cap_text->font_size = 30;
    ui_g_Ungroup_cap_text->number = 100;

    ui_g_Ungroup_aim->figure_type = 7;
    ui_g_Ungroup_aim->operate_type = 1;
    ui_g_Ungroup_aim->layer = 0;
    ui_g_Ungroup_aim->color = 2;
    ui_g_Ungroup_aim->start_x = 850;
    ui_g_Ungroup_aim->start_y = 460;
    ui_g_Ungroup_aim->width = 3;
    ui_g_Ungroup_aim->font_size = 30;
    ui_g_Ungroup_aim->str_length = 8;
    strcpy(ui_g_Ungroup_aim->string, "DETECTED");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
