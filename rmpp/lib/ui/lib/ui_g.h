//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_round_t *ui_g_Ungroup_yaw_round;
extern ui_interface_number_t *ui_g_Ungroup_bullet_speed_2;
extern ui_interface_line_t *ui_g_Ungroup_yaw;
extern ui_interface_arc_t *ui_g_Ungroup_hit;
extern ui_interface_number_t *ui_g_Ungroup_enemy_hp;
extern ui_interface_number_t *ui_g_Ungroup_shoot_current;
extern ui_interface_number_t *ui_g_Ungroup_bullet_speed_1;
extern ui_interface_string_t *ui_g_Ungroup_aim;

void ui_init_g_Ungroup();
void ui_update_g_Ungroup();
void ui_remove_g_Ungroup();


#endif // UI_g_H
