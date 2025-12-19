#include "ui.hpp"
#include "referee/referee.hpp"

// 适配rmui
extern "C" void print_message(const uint8_t* message, const int length) {
    extern Referee referee;
    referee.AddCanData(message, length);
}

extern "C" void _ui_init_g_Ungroup_0();
extern "C" void _ui_init_g_Ungroup_1();

extern "C" void _ui_update_g_Ungroup_0();
extern "C" void _ui_update_g_Ungroup_1();

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
                _ui_init_g_Ungroup_0();
                state = 2;
                break;
            case 2:
                _ui_init_g_Ungroup_1();
                state = 3;
                break;
            case 3:
                _ui_update_g_Ungroup_0();
                state = 4;
                break;
            case 4:
                _ui_update_g_Ungroup_1();
                state = 3;
                break;
            default:
                break;
        }
    }
}

void UI::updateLib() {
    // aim
    if (is_detected) {
        ui_g_Ungroup_aim->color = 3; // 橙色
        strcpy(ui_g_Ungroup_aim->string, "DETECT");
    } else if (is_mavlink_connected) {
        ui_g_Ungroup_aim->color = 2; // 绿色
        strcpy(ui_g_Ungroup_aim->string, "ONLINE");
    } else {
        strcpy(ui_g_Ungroup_aim->string, "      ");
    }

    // dir
    // 高中平面直角坐标系
    const int dx = (int)(unit::cos(gimbal_yaw + 90 * deg).toFloat() * 50);
    const int dy = (int)(unit::sin(gimbal_yaw + 90 * deg).toFloat() * 50);
    ui_g_Ungroup_dir->end_x = ui_g_Ungroup_dir->start_x + dx;
    ui_g_Ungroup_dir->end_y = ui_g_Ungroup_dir->start_y + dy;

    // hit
    if (is_hit) {
        // 6点钟方向为0度，逆时针增加
        // ±18°逆时针画弧
        int start_angle = (int)(Angle(-hit - 18 * deg).toFloat(deg));
        int end_angle = (int)(Angle(-hit + 18 * deg).toFloat(deg));
        if (start_angle < 0) start_angle += 360;
        if (end_angle < 0) end_angle += 360;
        ui_g_Ungroup_hit->width = 20;
        ui_g_Ungroup_hit->start_angle = start_angle;
        ui_g_Ungroup_hit->end_angle = end_angle;
    } else {
        ui_g_Ungroup_hit->width = 0;
        ui_g_Ungroup_hit->start_angle = 0;
        ui_g_Ungroup_hit->end_angle = 0;
    }
}