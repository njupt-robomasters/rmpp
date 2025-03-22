#include "ui.hpp"
#include "ui.h"
#include <cstring>
#include "app_variable.hpp"

void UI::Init(const uint8_t robot_id) {
    ui_self_id = robot_id;
    ui_init_g_Ungroup();
}

void UI::Update() {
    char ui_buf[30];
    // id 0 shoot freq
    sprintf(ui_buf, "%dhz", this->set_bullet_frequency);
    strcpy(ui_g_Ungroup_shooter_freq->string, ui_buf);
    _ui_update_g_Ungroup_0();

    // id 1 auto_aim_status
    if (set_is_firing) {
        strcpy(ui_g_Ungroup_shooter_freq->string, "Firing");
        ui_g_Ungroup_auto_aim_status->color = 3; //orange
        _ui_update_g_Ungroup_1();
    } else if (set_is_locked) {
        strcpy(ui_g_Ungroup_shooter_freq->string, "Locked");
        ui_g_Ungroup_auto_aim_status->color = 2; //green
        _ui_update_g_Ungroup_1();
    } else {
        _ui_remove_g_Ungroup_1();
    }

    // id 2 chassis mode
    if (set_gimbal_is_imu_mode) {
        strcpy(ui_g_Ungroup_chassis_mode->string, "Q: IMU_Mode");
        ui_g_Ungroup_chassis_mode->color = 2; //green
    } else {
        strcpy(ui_g_Ungroup_chassis_mode->string, "Q: ECD_Mode");
        ui_g_Ungroup_chassis_mode->color = 3; //orange
    }
    _ui_update_g_Ungroup_2();

    // id 3 chassis_mode
    if (set_chassis_mode == UI::TURNING) {
        strcpy(ui_g_Ungroup_auto_aim_status->string, "F: TURNING");
        _ui_update_g_Ungroup_3();
    } else {
        strcpy(ui_g_Ungroup_auto_aim_status->string, "F: FOLLOW");
        _ui_update_g_Ungroup_3();
    }

    // id 4 chassis_vr
    sprintf(ui_buf, "%d rpm", set_chassis_vr);
    strcpy(ui_g_Ungroup_chassis_vr->string, ui_buf);
    _ui_update_g_Ungroup_4();

    // id 5
    sprintf(ui_buf, "%d m/s", set_chassis_max_speed);
    strcpy(ui_g_Ungroup_chassis_speed->string, ui_buf);
    _ui_update_g_Ungroup_5();

    // id 6 7 update
    switch (set_center_gain_status) {
        case 0:
            _ui_remove_g_Ungroup_6();
            _ui_remove_g_Ungroup_7();
            break;
        case 1:
            ui_g_Ungroup_our_gain_status->font_size = 25; //防止不更新
            _ui_update_g_Ungroup_6();
            _ui_remove_g_Ungroup_7();
            break;
        case 2:
            ui_g_Ungroup_enemy_gain_status->font_size = 25;
            _ui_update_g_Ungroup_7();
            _ui_remove_g_Ungroup_6();
            break;
        case 3:
            ui_g_Ungroup_enemy_gain_status->font_size = 25;
            ui_g_Ungroup_our_gain_status->font_size = 25;
            _ui_update_g_Ungroup_6();
            _ui_update_g_Ungroup_7();
            break;

        default:
            break;
    }

    // id 8 superCap status
    sprintf(ui_buf, "%.1f", set_super_cap_percent);
    strcpy(ui_g_Ungroup_super_cap_status->string, ui_buf);
    _ui_update_g_Ungroup_8();

    // id 9 power_limit
    sprintf(ui_buf, "%d W", referee.chassis_power_limit);
    strcpy(ui_g_Ungroup_power_limit->string, ui_buf);
    _ui_update_g_Ungroup_9();

    // id 10 摩擦轮电机状态
    if (set_shooter_is_on) {
        strcpy(ui_g_Ungroup_shooter_status->string, "AM ON");
        ui_g_Ungroup_shooter_status->color = 2; //green
    } else {
        strcpy(ui_g_Ungroup_shooter_status->string, "AM OFF!");
        ui_g_Ungroup_shooter_status->color = 3; //orange
    }
    _ui_update_g_Ungroup_10();
}
