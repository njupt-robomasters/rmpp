#include "UI.hpp"
#include <algorithm>
#include <cstring>
#include "lib/ui.h"

// 适配rmui
extern "C" void print_message(const uint8_t* message, const int length) {
    UI::AddCanData(message, length);
}

static void (*ui_init_g[])() = {
    ui_init_g_00,
    ui_init_g_01,
    ui_init_g_02,
    ui_init_g_03,
    ui_init_g_04,
    ui_init_g_05,

    ui_init_g_1,
    ui_init_g_2,
    ui_init_g_3,
};

// CAN发送队列
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
    state = INIT;
    init_index = 0;
}

void UI::OnLoop() {
    updateLib(); // 更新UI库

    if (state == INIT) {
        if (dwt.PollTimeout(1 / INIT_FREQ)) { // 发送频率控制
            ui_init_g[init_index]();
            init_index++;
            if (init_index >= std::size(ui_init_g)) {
                state = UPDATE;
            }
        }
    } else if (state == UPDATE) {
        if (dwt.PollTimeout(1 / UPDATE_FREQ)) { // 发送频率控制
            update_cnt++;
            if (update_cnt % 10 == 0) {
                ui_update_g_2();
            } else{
                ui_update_g_1();
            }
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
    int start_angle, end_angle, angle;

    // 云台相对底盘角度
    // 0°指12点钟方向，顺时针绘制
    start_angle = (int)Angle(yaw_ecd - 20 * deg).toFloat(deg);
    end_angle = (int)Angle(yaw_ecd + 20 * deg).toFloat(deg);
    if (start_angle < 0) start_angle += 360;
    if (end_angle < 0) end_angle += 360;
    ui_g_1_dir->start_angle = start_angle;
    ui_g_1_dir->end_angle = end_angle;

    // 云台角度
    ui_g_2_yaw->number = (int32_t)yaw.toFloat(deg);
    ui_g_2_pitch->number = (int32_t)pitch.toFloat(deg);

    // 伤害方向
    if (is_hurt) {
        start_angle = (int)Angle(-hurt_dir - 20 * deg).toFloat(deg);
        end_angle = (int)Angle(-hurt_dir + 20 * deg).toFloat(deg);
        if (start_angle < 0) start_angle += 360;
        if (end_angle < 0) end_angle += 360;
        ui_g_1_hurt->width = 10;
        ui_g_1_hurt->start_angle = start_angle;
        ui_g_1_hurt->end_angle = end_angle;
    } else {
        ui_g_1_hurt->width = 0;
    }

    // 电容剩余能量比例
    // 275°~313°
    angle = (int)((cap_ratio / config.max_cap_ratio).toFloat() * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar0->start_angle = 275;
    ui_g_1_bar0->end_angle = 275 + angle;
    ui_g_2_number0->number = (int32_t)cap_ratio.toFloat(pct);

    // 底盘旋转速度
    // 227°~265°
    angle = (int)((unit::abs(chassis_wr) / config.max_chassis_wr).toFloat() * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar1->start_angle = 265 - angle;
    ui_g_1_bar1->end_angle = 265;
    ui_g_2_number1->number = (int32_t)chassis_wr.toFloat(rpm);

    // 弹频
    // 95°~133°
    angle = (int)((shoot_freq / config.max_shoot_freq).toFloat() * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar2->start_angle = 95;
    ui_g_1_bar2->end_angle = 95 + angle;
    ui_g_2_number2->number = (int32_t)shoot_freq.toFloat(Hz);

    // 拨弹电机电流
    // 47°~85°
    angle = (int)((unit::abs(shoot_current) / config.max_shoot_current).toFloat() * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar3->start_angle = 85 - angle;
    ui_g_1_bar3->end_angle = 85;
    ui_g_2_number3->number = (int32_t)shoot_current.toFloat(A);

    // 自瞄状态
    if (aim.is_detect) {
        ui_g_3_aim_status->color = 3; // 橙色
        std::strcpy(ui_g_3_aim_status->string, "DETECT");
    } else if (aim.is_connect) {
        ui_g_3_aim_status->color = 2; // 绿色
        std::strcpy(ui_g_3_aim_status->string, "ONLINE");
    } else {
        std::strcpy(ui_g_3_aim_status->string, "      ");
    }
}