#include "ui.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include "rmui_adapter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

// CAN发送队列
uint8_t Ui::txbuf[TXBUF_SIZE];
size_t Ui::head, Ui::tail;

Ui::Ui(const Config& config)
    : config(config) {}

void Ui::add_can_data(const uint8_t data[], const size_t len) {
    for (size_t i = 0; i < len; i++) {
        if ((tail + 1) % TXBUF_SIZE == head)
            break;                      // 队列满
        txbuf[tail] = data[i];          // 拷贝数据
        tail = (tail + 1) % TXBUF_SIZE; // 后移队尾指针
    }
}

void Ui::init() {
    state_ = INIT;
    init_index_ = 0;
    update_index_ = 0;
}

void Ui::update() {
    update_rmui_robot_id();

    update_lib(); // 更新UI库

    if (timer_.elapsed() > 1 / SEND_FREQ) { // 发送频率控制
        timer_.restart();
        if (state_ == INIT) {
            ui_init_g[init_index_]();
            init_index_++;
            if (init_index_ >= std::size(ui_init_g)) {
                state_ = UPDATE;
            }
        } else if (state_ == UPDATE) {
            ui_update_g[update_index_]();
            update_index_++;
            if (update_index_ >= std::size(ui_update_g)) {
                update_index_ = 0;
            }
        }
    }
}

void Ui::send_can_cmd() {
    if (head != tail) { // 队列有数据
        CanFrame frame{.id = config.slave_id};
        auto next_head = head;
        for (int i = 0; i < 8; i++) {
            if (next_head == tail)
                break;
            frame.data[i] = txbuf[next_head];
            next_head = (next_head + 1) % TXBUF_SIZE;
            frame.size++;
        }
        config.can.send(frame);
        head = next_head;
    }
}

void Ui::update_lib() const {
    int start_angle, end_angle, angle;

    // 云台相对底盘角度
    // 0°指12点钟方向，顺时针绘制
    start_angle = static_cast<int>(Angle(yaw_ecd - 20 * deg) / deg);
    end_angle = static_cast<int>(Angle(yaw_ecd + 20 * deg) / deg);
    if (start_angle < 0)
        start_angle += 360;
    if (end_angle < 0)
        end_angle += 360;
    ui_g_1_dir->start_angle = start_angle;
    ui_g_1_dir->end_angle = end_angle;

    // 云台IMU角度
    ui_g_2_yaw->number = static_cast<int32_t>(yaw / deg);
    ui_g_2_pitch->number = static_cast<int32_t>(pitch / deg);

    // 伤害方向
    if (is_hurt) {
        start_angle = static_cast<int>(Angle(-hurt_dir - 20 * deg) / deg);
        end_angle = static_cast<int>(Angle(-hurt_dir + 20 * deg) / deg);
        if (start_angle < 0)
            start_angle += 360;
        if (end_angle < 0)
            end_angle += 360;
        ui_g_1_hurt->width = 10;
        ui_g_1_hurt->start_angle = start_angle;
        ui_g_1_hurt->end_angle = end_angle;
    } else {
        ui_g_1_hurt->width = 0;
    }

    // 自瞄识别
    if (is_detect) {
        ui_g_1_detect->width = 21;
    } else {
        ui_g_1_detect->width = 0;
    }

    // 电容剩余能量比例
    // 275°~313°
    angle = (int)((cap / config.max_cap) * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar0->start_angle = 275;
    ui_g_1_bar0->end_angle = 275 + angle;
    ui_g_2_number0->number = static_cast<int32_t>(cap / pct);

    // 底盘旋转速度
    // 227°~265°
    angle = (int)((std::fabsf(wr) / config.max_wr) * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar1->start_angle = 265 - angle;
    ui_g_1_bar1->end_angle = 265;
    ui_g_2_number1->number = static_cast<int32_t>(wr / rpm);

    // 英雄弹速/步兵弹频
    // 95°~133°
    angle = (int)((bullet / config.max_bullet) * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar2->start_angle = 95;
    ui_g_1_bar2->end_angle = 95 + angle;
    ui_g_2_number2->number = static_cast<int32_t>(bullet * (bullet_is_speed ? 10.0f : 1.0f));

    // 拨弹电机电流
    // 47°~85°
    angle = (int)((std::fabsf(shoot) / config.max_shoot) * 38);
    angle = std::clamp(angle, 1, 38);
    ui_g_1_bar3->start_angle = 85 - angle;
    ui_g_1_bar3->end_angle = 85;
    ui_g_2_number3->number = static_cast<int32_t>(shoot);

    // 机器人状态
    ui_g_30_w1->color = robot.w1;
    ui_g_30_w2->color = robot.w2;
    ui_g_30_w3->color = robot.w3;
    ui_g_30_w4->color = robot.w4;
    ui_g_30_s1->color = robot.s1;
    ui_g_30_s2->color = robot.s2;
    ui_g_30_s3->color = robot.s3;

    ui_g_31_s4->color = robot.s4;
    ui_g_31_cap->color = robot.referee;
    ui_g_31_yaw1->color = robot.yaw1;
    ui_g_31_yaw2->color = robot.yaw2;
    ui_g_31_pitch->color = robot.pitch;
    ui_g_31_rub_left1->color = robot.rub1;
    ui_g_31_rub_right1->color = robot.rub2;

    ui_g_32_rub_left2->color = robot.rub3;
    ui_g_32_rub_right2->color = robot.rub4;
    ui_g_32_shoot->color = robot.shoot;
    ui_g_32_aim->color = robot.aim;
}
