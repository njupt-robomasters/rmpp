#include "ui.hpp"

#include <cstring>

void UI::Init(const uint8_t robot_id) {
    ui_self_id = robot_id;
    is_first_update = 1;
    ui_init_g_Ungroup();
}

uint8_t UI::Update() {
    uint8_t update_num = 0;
    char ui_buf[30];
    // id 0 shoot freq
    if (set_bullet_frequency != set_bullet_frequency_last || is_first_update) {
        sprintf(ui_buf, "%dhz", this->set_bullet_frequency);
        strcpy(ui_g_Ungroup_shooter_freq->string, ui_buf);
        _ui_update_g_Ungroup_0();
        update_num++;
    }

    // id 1 auto_aim_status
    if (set_is_firing != set_is_firing_last || set_is_locked != set_is_locked_last || is_first_update) {
        if (set_is_firing) {
            strcpy(ui_g_Ungroup_auto_aim_status->string, "Firing");
            ui_g_Ungroup_auto_aim_status->color = 3; //orange
            show_string(ui_g_Ungroup_auto_aim_status);
        } else if (set_is_locked) {
            strcpy(ui_g_Ungroup_auto_aim_status->string, "Locked");
            ui_g_Ungroup_auto_aim_status->color = 2; //green
            show_string(ui_g_Ungroup_auto_aim_status);
        } else {
            hide_string(ui_g_Ungroup_auto_aim_status);
        }
        _ui_update_g_Ungroup_1();
        update_num++;
    }


    // id 2 chassis mode
    if (set_gimbal_is_imu_mode != set_gimbal_is_imu_mode_last || is_first_update) {
        if (set_gimbal_is_imu_mode) {
            strcpy(ui_g_Ungroup_chassis_mode->string, "(Q) IMU_Mode");
            ui_g_Ungroup_chassis_mode->color = 2; //green
        } else {
            strcpy(ui_g_Ungroup_chassis_mode->string, "(Q) ECD_Mode");
            ui_g_Ungroup_chassis_mode->color = 3; //orange
        }
        _ui_update_g_Ungroup_2();
        update_num++;
    }


    // id 3 chassis_mode
    if (set_chassis_mode != set_chassis_mode_last || is_first_update) {
        if (set_chassis_mode == UI::TURNING)
            strcpy(ui_g_Ungroup_chassis_around_status->string, "(F) TURNING");
        else
            strcpy(ui_g_Ungroup_chassis_around_status->string, "(F) FOLLOW");
        _ui_update_g_Ungroup_3();
        update_num++;
    }

    // id 4 chassis_vr
    if (set_chassis_vr != set_chassis_vr_last || is_first_update) {
        sprintf(ui_buf, "%d rpm", set_chassis_vr);
        strcpy(ui_g_Ungroup_chassis_vr->string, ui_buf);
        _ui_update_g_Ungroup_4();
        update_num++;
    }

    // id 5
    if (set_chassis_max_speed != set_chassis_max_speed_last || is_first_update) {
        sprintf(ui_buf, "%d m/s", set_chassis_max_speed);
        strcpy(ui_g_Ungroup_chassis_speed->string, ui_buf);
        _ui_update_g_Ungroup_5();
        update_num++;
    }


    // id 6 7 update
    if (set_center_gain_status != set_center_gain_status_last || is_first_update) {
        switch (set_center_gain_status) {
            case 0:
                hide_string(ui_g_Ungroup_our_gain_status);
                hide_string(ui_g_Ungroup_enemy_gain_status);
                break;
            case 1:
                show_string(ui_g_Ungroup_our_gain_status);
                hide_string(ui_g_Ungroup_enemy_gain_status);
                break;
            case 2:
                hide_string(ui_g_Ungroup_our_gain_status);
                show_string(ui_g_Ungroup_enemy_gain_status);
                break;
            case 3:
                show_string(ui_g_Ungroup_our_gain_status);
                show_string(ui_g_Ungroup_enemy_gain_status);
                break;

            default:
                break;
        }
        _ui_update_g_Ungroup_6();
        _ui_update_g_Ungroup_7();
        update_num++;
        update_num++;
        update_num++;
    }


    // id 8 superCap status
    if (set_super_cap_percent != set_super_cap_percent_last || is_first_update) {
        sprintf(ui_buf, "%.1f %%", set_super_cap_percent * 100);
        strcpy(ui_g_Ungroup_super_cap_status->string, ui_buf);
        _ui_update_g_Ungroup_8();
        update_num++;
    }


    // id 9 power_limit
    if (set_chassis_power_limit != set_chassis_power_limit_last || is_first_update) {
        sprintf(ui_buf, "%d W", set_chassis_power_limit);
        strcpy(ui_g_Ungroup_power_limit->string, ui_buf);
        _ui_update_g_Ungroup_9();
        update_num++;
    }

    // id 10 摩擦轮电机状态
    if (set_shooter_is_on != set_shooter_is_on_last || is_first_update) {
        if (set_shooter_is_on) {
            strcpy(ui_g_Ungroup_shooter_status->string, "AM ON");
            ui_g_Ungroup_shooter_status->color = 2; //green
        } else {
            strcpy(ui_g_Ungroup_shooter_status->string, "AM OFF!");
            ui_g_Ungroup_shooter_status->color = 3; //orange
        }
        _ui_update_g_Ungroup_10();
        update_num++;
    }
    save_last();
    is_first_update = 0;
    return update_num;
}

void UI::show_string(ui_interface_string_t *ui_string) {
    ui_string->font_size = 25;
}

void UI::hide_string(ui_interface_string_t *ui_string) {
    ui_string->font_size = 0;
}

void UI::save_last() {
    set_bullet_frequency_last = set_bullet_frequency; // 弹丸发射频率
    set_gimbal_is_imu_mode_last = set_gimbal_is_imu_mode; // 云台模式，是imu模式为1，否则为0
    set_chassis_max_speed_last = set_chassis_max_speed; // 底盘平移速度
    set_is_locked_last = set_is_locked; // 自瞄是否锁定
    set_is_firing_last = set_is_firing; // 是否自动火控
    set_chassis_mode_last = set_chassis_mode; // 底盘模式
    set_chassis_vr_last = set_chassis_vr; // 底盘自转速度
    set_center_gain_status_last = set_center_gain_status; // 中心增益点占领情况
    set_super_cap_percent_last = set_super_cap_percent; // 超电百分比
    set_shooter_is_on_last = set_shooter_is_on; // 摩擦轮电机状态
}
