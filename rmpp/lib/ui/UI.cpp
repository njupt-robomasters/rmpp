#include "UI.hpp"
#include "lib/UI.h"

// 适配rmui
extern "C" void print_message(const uint8_t* message, const int length) {
    UI::AddCanData(message, length);
}

extern "C" void _ui_init_g_Ungroup_0();
extern "C" void _ui_init_g_Ungroup_1();

extern "C" void _ui_update_g_Ungroup_0();
extern "C" void _ui_update_g_Ungroup_1();

uint8_t UI::txbuf[TXBUF_SIZE];
size_t UI::head, UI::tail;

UI::UI(const config_t& config) : config(config) {}

void UI::AddCanData(const uint8_t data[], const size_t len) {
    for (size_t i = 0; i < len; i++) {
        if ((tail + 1) % TXBUF_SIZE == head) break; // 队列满
        txbuf[tail] = data[i];                      // 拷贝数据
        tail = (tail + 1) % TXBUF_SIZE;             // 后移队尾指针
    }
}

void UI::Init() {
    state = 0;
}

void UI::OnLoop() {
    if (dwt_update.PollTimeout(1 / config.update_freq)) {
        updateLib(); // 更新UI库

        switch (state) {
            case 0:
                ui_delete_layer(2, 0); // 删除所有图层
                state = 1;
                break;
            case 1:
                _ui_init_g_Ungroup_0(); // 初始化group_0
                state = 2;
                break;
            case 2:
                _ui_init_g_Ungroup_1(); // 初始化group_1
                state = 3;
                break;
            case 3:
                _ui_update_g_Ungroup_0(); // 更新group_0
                state = 4;
                break;
            case 4:
                _ui_update_g_Ungroup_1(); // 更新group_1
                state = 3;
                break;
            default:
                break;
        }
    }
}

void UI::SendCanCmd() {
    if (head != tail) { // 队列有数据
        uint8_t data[8];
        uint8_t dlc = 0;
        for (int i = 0; i < 8; i++) {
            if (head == tail) break;        // 队列空
            data[i] = txbuf[head];          // 拷贝数据
            head = (head + 1) % TXBUF_SIZE; // 后移队首指针
            dlc++;
        }
        BSP::CAN::TransmitStd(config.can_port, config.slave_id, data, dlc);
    }
}

void UI::updateLib() const {
    // 云台yaw角
    // 高中平面直角坐标系
    const int dx = (int)(unit::cos(yaw + 90 * deg).toFloat() * 50);
    const int dy = (int)(unit::sin(yaw + 90 * deg).toFloat() * 50);
    ui_g_Ungroup_yaw->end_x = ui_g_Ungroup_yaw->start_x + dx;
    ui_g_Ungroup_yaw->end_y = ui_g_Ungroup_yaw->start_y + dy;

    // 伤害方向提示
    if (is_hurt) {
        int start_angle = (int)(Angle(-hurt_dir - 18 * deg).toFloat(deg));
        int end_angle = (int)(Angle(-hurt_dir + 18 * deg).toFloat(deg));
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

    // 摩擦轮速度
    ui_g_Ungroup_bullet_speed_1->number = (int32_t)(bullet_speed_1.toFloat(m_s) * 1000);
    ui_g_Ungroup_bullet_speed_2->number = (int32_t)(bullet_speed_2.toFloat(m_s) * 1000);

    // 拨弹电机电流
    ui_g_Ungroup_shoot_current->number = (int32_t)(shoot_current.toFloat(A) * 1000);

    // 自瞄状态
    if (aim.is_detect) {
        ui_g_Ungroup_aim->color = 3; // 橙色
        strcpy(ui_g_Ungroup_aim->string, "DETECT");
    } else if (aim.is_connect) {
        ui_g_Ungroup_aim->color = 2; // 绿色
        strcpy(ui_g_Ungroup_aim->string, "ONLINE");
    } else {
        strcpy(ui_g_Ungroup_aim->string, "      ");
    }
}