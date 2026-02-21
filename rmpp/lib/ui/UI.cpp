#include "UI.hpp"
#include <algorithm>
#include <cstring>
#include "rmui_adapter.hpp"

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
    update_index = 0;
}

void UI::OnLoop() {
    update_rmui_robot_id();

    updateLib(); // 更新UI库

    if (dwt.PollTimeout(1 / SEND_FREQ)) { // 发送频率控制
        if (state == INIT) {
            ui_init_g[init_index]();
            init_index++;
            if (init_index >= std::size(ui_init_g)) {
                state = UPDATE;
            }
        } else if (state == UPDATE) {
            ui_update_g[update_index]();
            update_index++;
            if (update_index >= std::size(ui_update_g)) {
                update_index = 0;
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

    // 云台IMu角度
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

    // 自瞄识别
    if (is_detect) {
        ui_g_1_detect->width = 21;
    } else {
        ui_g_1_detect->width = 0;
    }

    // 整车状态
    ui_g_30_w1->color = robot.w1;
    ui_g_30_w2->color = robot.w2;
    ui_g_30_w3->color = robot.w3;
    ui_g_30_w4->color = robot.w4;
    ui_g_30_s1->color = robot.s1;
    ui_g_30_s2->color = robot.s2;
    ui_g_30_s3->color = robot.s3;

    ui_g_31_s4->color = robot.s4;
    ui_g_31_cap->color = robot.cap;
    ui_g_31_yaw1->color = robot.yaw1;
    ui_g_31_yaw2->color = robot.yaw2;
    ui_g_31_pitch->color = robot.pitch;
    ui_g_31_rub_left1->color = robot.rub_left1;
    ui_g_31_rub_right1->color = robot.rub_right1;

    ui_g_32_rub_left2->color = robot.rub_left2;
    ui_g_32_rub_right2->color = robot.rub_right2;
    ui_g_32_shoot->color = robot.shoot;
    ui_g_32_aim->color = robot.aim;
}