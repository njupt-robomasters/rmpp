#include "ui.hpp"

void UI::Init() {
    ui_init_g();
}

void UI::OnLoop() {
    if (dwt.GetDT() > 1 / UPDATE_FREQ.toFloat(Hz)) {
        dwt.UpdateDT();

        // aim
        if (is_locked) {
            ui_g_Ungroup_aim->str_length = 4;
            strcpy(ui_g_Ungroup_aim->string, "LOCK");
        } else if (is_detected) {
            ui_g_Ungroup_aim->str_length = 6;
            strcpy(ui_g_Ungroup_aim->string, "DETECT");
        } else {
            ui_g_Ungroup_aim->str_length = 0;
            strcpy(ui_g_Ungroup_aim->string, "");
        }

        // dir
        // 高中平面直角坐标系
        const int dx = (int)(unit::cos(-yaw + 90 * deg).toFloat() * 50);
        const int dy = (int)(unit::sin(-yaw + 90 * deg).toFloat() * 50);
        ui_g_Ungroup_dir->end_x = ui_g_Ungroup_dir->start_x + dx;
        ui_g_Ungroup_dir->end_y = ui_g_Ungroup_dir->start_y + dy;

        // cap
        const int width = (int)(cap.toFloat() * (1210 - 710));
        ui_g_Ungroup_cap_rect->end_x = ui_g_Ungroup_cap_rect->start_x + width;
        const int number = (int)cap.toFloat(pct);
        ui_g_Ungroup_cap_text->number = number;

        // hit
        // 6点钟方向为0度，逆时针增加
        // ±18°逆时针画弧
        int start_angle = (int)(Angle(hit + 180 * deg - 18 * deg).toFloat(deg));
        if (start_angle < 0) start_angle += 360;
        int end_angle = (int)(Angle(hit + 180 * deg + 18 * deg).toFloat(deg));
        if (end_angle < 0) end_angle += 360;
        ui_g_Ungroup_hit->start_angle = start_angle;
        ui_g_Ungroup_hit->end_angle = end_angle;

        ui_update_g();
    }
}

