#include "ui.hpp"

void UI::Init() {
    state = 0;
}

void UI::OnLoop() {
    if (dwt_update_freq.GetDT() > 1 / UPDATE_FREQ.toFloat(Hz)) {
        dwt_update_freq.UpdateDT();

        updateLib();

        switch (state) {
            case 0:
                ui_delete_layer(2, 0);
                state = 1;
                break;
            case 1:
                ui_init_g_Ungroup();
                state = 2;
                break;
            case 2:
                ui_update_g_Ungroup();
                break;
            default:
                break;
        }
    }
}

void UI::updateLib() {
    // aim
    if (is_locked) {
        strcpy(ui_g_Ungroup_aim->string, " LOCK ");
    } else if (is_detected) {
        strcpy(ui_g_Ungroup_aim->string, "DETECT");
    } else {
        strcpy(ui_g_Ungroup_aim->string, "      ");
    }

    // dir
    // 高中平面直角坐标系
    const int dx = (int)(unit::cos(-yaw + 90 * deg).toFloat() * 50);
    const int dy = (int)(unit::sin(-yaw + 90 * deg).toFloat() * 50);
    ui_g_Ungroup_dir->end_x = ui_g_Ungroup_dir->start_x + dx;
    ui_g_Ungroup_dir->end_y = ui_g_Ungroup_dir->start_y + dy;

    // hit
    // 6点钟方向为0度，逆时针增加
    // ±18°逆时针画弧
    int start_angle = (int)(Angle(hit + 180 * deg - 18 * deg).toFloat(deg));
    if (start_angle < 0) start_angle += 360;
    int end_angle = (int)(Angle(hit + 180 * deg + 18 * deg).toFloat(deg));
    if (end_angle < 0) end_angle += 360;
    ui_g_Ungroup_hit->width = 20;
    ui_g_Ungroup_hit->start_angle = 0;
    ui_g_Ungroup_hit->end_angle = 0;
}