#include "robot_ui.hpp"
#include <algorithm>
#include <cmath>

RobotUi::RobotUi(const Config& config)
    : config(config) {}

void RobotUi::init(RefereeSender& sender) {
    sender.clear_ui();

    // 图层 0 准星：十字四段
    const Figure cross[] = {
        RefereeSender::line(0x000100, 0, WHITE, 2, 960, 660, 960, 890),
        RefereeSender::line(0x000101, 0, WHITE, 2, 840, 540, 610, 540),
        RefereeSender::line(0x000102, 0, WHITE, 2, 960, 420, 960, 190),
        RefereeSender::line(0x000103, 0, WHITE, 2, 1080, 540, 1310, 540),
    };
    sender.draw(cross, 4, RefereeSender::ADD);

    // 图层 0 提示条边界：四条提示条两端的短弧，度
    const Figure limits[] = {
        RefereeSender::arc(0x000110, 0, WHITE, 18, 960, 540, 315 * deg, 317 * deg, 387, 387),
        RefereeSender::arc(0x000111, 0, WHITE, 18, 960, 540, 267 * deg, 273 * deg, 387, 387),
        RefereeSender::arc(0x000112, 0, WHITE, 18, 960, 540, 223 * deg, 225 * deg, 387, 387),
        RefereeSender::arc(0x000113, 0, WHITE, 18, 960, 540, 135 * deg, 137 * deg, 387, 387),
        RefereeSender::arc(0x000114, 0, WHITE, 18, 960, 540, 87 * deg, 93 * deg, 387, 387),
        RefereeSender::arc(0x000115, 0, WHITE, 18, 960, 540, 43 * deg, 45 * deg, 387, 387),
    };
    sender.draw(limits, 6, RefereeSender::ADD);

    // 图层 0 提示条标签：左上 CAP、左下 RPM、右下 BULLET、右上 SHOOT
    sender.draw_text(RefereeSender::character(0x000120, 0, WHITE, 2, 660, 860, 20), "CAP", 3, RefereeSender::ADD);
    sender.draw_text(RefereeSender::character(0x000130, 0, WHITE, 2, 660, 240, 20), "RPM", 3, RefereeSender::ADD);
    sender.draw_text(RefereeSender::character(0x000140, 0, WHITE, 2, 1200, 240, 20), "BULLET", 6, RefereeSender::ADD);
    sender.draw_text(RefereeSender::character(0x000150, 0, WHITE, 2, 1200, 860, 20), "SHOOT", 5, RefereeSender::ADD);

    // 图层 1 仪表弧与准星圆，update() 修改
    const Figure arcs[] = {
        RefereeSender::arc(0x000160, 1, GREEN, 10, 960, 540, 0 * deg, 360 * deg, 85, 85),
        RefereeSender::arc(0x000161, 1, PINK, 0, 960, 540, 0 * deg, 360 * deg, 85, 85),
        RefereeSender::arc(0x000162, 1, GREEN, 14, 960, 540, 275 * deg, 313 * deg, 387, 387),
        RefereeSender::arc(0x000163, 1, GREEN, 14, 960, 540, 227 * deg, 265 * deg, 387, 387),
        RefereeSender::arc(0x000164, 1, GREEN, 14, 960, 540, 95 * deg, 133 * deg, 387, 387),
        RefereeSender::arc(0x000165, 1, GREEN, 14, 960, 540, 47 * deg, 85 * deg, 387, 387),
        RefereeSender::circle(0x000166, 1, GREEN, 2, 960, 540, 21),
    };
    sender.draw(arcs, 7, RefereeSender::ADD);

    // 图层 2 数值，update() 修改
    const Figure numbers[] = {
        RefereeSender::integer(0x000170, 2, WHITE, 2, 615, 565, 15, 0),
        RefereeSender::integer(0x000171, 2, WHITE, 2, 615, 530, 15, 0),
        RefereeSender::integer(0x000172, 2, WHITE, 2, 1250, 530, 15, 0),
        RefereeSender::integer(0x000173, 2, WHITE, 2, 1250, 565, 15, 0),
        RefereeSender::integer(0x000174, 2, WHITE, 2, 970, 680, 15, 0),
        RefereeSender::integer(0x000175, 2, WHITE, 2, 780, 565, 15, 0),
    };
    sender.draw(numbers, 6, RefereeSender::ADD);

    draw_car(sender, RefereeSender::ADD);
}

void RobotUi::draw_car(RefereeSender& sender, const RefereeSender::Operation operation) {
    // 图层 3 整车示意图：轮子
    const Figure wheel[] = {
        RefereeSender::arc(0x003000, 3, operation == RefereeSender::ADD ? WHITE : w1, 2, 290, 551, 90 * deg, 217 * deg, 30, 25),
        RefereeSender::arc(0x003001, 3, operation == RefereeSender::ADD ? WHITE : w2, 2, 142, 551, 90 * deg, 217 * deg, 30, 25),
        RefereeSender::arc(0x003002, 3, operation == RefereeSender::ADD ? WHITE : w3, 2, 100, 546, 90 * deg, 270 * deg, 30, 25),
        RefereeSender::arc(0x003003, 3, operation == RefereeSender::ADD ? WHITE : w4, 2, 248, 545, 90 * deg, 270 * deg, 30, 25),
        RefereeSender::rectangle(0x003004, 3, operation == RefereeSender::ADD ? WHITE : s1, 2, 280, 551, 320, 571),
        RefereeSender::rectangle(0x003005, 3, operation == RefereeSender::ADD ? WHITE : s2, 2, 132, 551, 172, 571),
        RefereeSender::rectangle(0x003006, 3, operation == RefereeSender::ADD ? WHITE : s3, 2, 70, 546, 130, 571),
    };
    sender.draw(wheel, 7, operation);

    // 图层 3 整车示意图：车身与云台
    const Figure modules[] = {
        RefereeSender::rectangle(0x003100, 3, operation == RefereeSender::ADD ? WHITE : s4, 2, 218, 546, 278, 571),
        RefereeSender::rectangle(0x003101, 3, operation == RefereeSender::ADD ? WHITE : cap, 2, 163, 588, 223, 608),
        RefereeSender::rectangle(0x003102, 3, operation == RefereeSender::ADD ? WHITE : yaw1, 2, 56, 573, 331, 623),
        RefereeSender::rectangle(0x003103, 3, operation == RefereeSender::ADD ? WHITE : yaw2, 2, 116, 639, 241, 689),
        RefereeSender::rectangle(0x003104, 3, operation == RefereeSender::ADD ? WHITE : pitch1, 2, 243, 648, 323, 678),
        RefereeSender::rectangle(0x003105, 3, operation == RefereeSender::ADD ? WHITE : rub_left_stage1, 2, 257, 664, 269, 676),
        RefereeSender::rectangle(0x003106, 3, operation == RefereeSender::ADD ? WHITE : rub_right_stage1, 2, 257, 650, 269, 662),
        RefereeSender::rectangle(0x003107, 3, operation == RefereeSender::ADD ? WHITE : pitch2, 2, 243, 680, 283, 690),
    };
    sender.draw(modules, 7, operation);

    const Figure modules_tail[] = {
        RefereeSender::rectangle(0x003200, 3, operation == RefereeSender::ADD ? WHITE : rub_left_stage2, 2, 271, 664, 283, 676),
        RefereeSender::rectangle(0x003201, 3, operation == RefereeSender::ADD ? WHITE : rub_right_stage2, 2, 271, 650, 283, 662),
        RefereeSender::rectangle(0x003202, 3, operation == RefereeSender::ADD ? WHITE : shoot, 2, 156, 625, 231, 637),
        RefereeSender::rectangle(0x003203, 3, operation == RefereeSender::ADD ? WHITE : aim, 2, 298, 680, 323, 692),
    };
    sender.draw(modules_tail, 4, operation);
}

void RobotUi::update(RefereeSender& sender) {
    // 提示条：满量程比例映射为 1~39 度弧长
    const auto span = [](const float ratio) { return static_cast<uint16_t>(1 + std::clamp(ratio, 0.0f, 1.0f) * 38); };
    const auto cap_span = span(super_cap / config.super_cap_max);
    const auto wr_span = span(std::fabs(chassis_wr) / config.chassis_wr_max);
    const auto bullet_span = span(bullet_freq / config.bullet_freq_max);
    const auto shoot_span = span(std::fabs(shoot_output) / config.shoot_output_max);

    // 图层 1：云台朝向 ±20 度弧、受击方向弧、四角提示条、识别时准星圆（自瞄在线色）
    const Figure arcs[] = {
        RefereeSender::arc(0x000160, 1, GREEN, 10, 960, 540, yaw_ecd - 20 * deg, yaw_ecd + 20 * deg, 85, 85),
        RefereeSender::arc(0x000161, 1, PINK, is_hurt ? 10 : 0, 960, 540, -hurt_dir - 20 * deg, -hurt_dir + 20 * deg, 85, 85),
        RefereeSender::arc(0x000162, 1, GREEN, 14, 960, 540, 275 * deg, (275 + cap_span) * deg, 387, 387),
        RefereeSender::arc(0x000163, 1, GREEN, 14, 960, 540, (265 - wr_span) * deg, 265 * deg, 387, 387),
        RefereeSender::arc(0x000164, 1, GREEN, 14, 960, 540, 95 * deg, (95 + bullet_span) * deg, 387, 387),
        RefereeSender::arc(0x000165, 1, GREEN, 14, 960, 540, (85 - shoot_span) * deg, 85 * deg, 387, 387),
        RefereeSender::circle(0x000166, 1, aim, is_aim_detect ? 21 : 0, 960, 540, 21),
    };
    sender.draw(arcs, 7, RefereeSender::MODIFY);

    // 图层 2：电容电压 V、转速 rpm、弹频 Hz、拨弹电流 A、yaw 度、pitch 度
    const Figure numbers[] = {
        RefereeSender::integer(0x000170, 2, WHITE, 2, 615, 565, 15, static_cast<int32_t>(std::lround(super_cap * 100))),
        RefereeSender::integer(0x000171, 2, WHITE, 2, 615, 530, 15, static_cast<int32_t>(chassis_wr / rpm)),
        RefereeSender::integer(0x000172, 2, WHITE, 2, 1250, 530, 15, static_cast<int32_t>(bullet_freq)),
        RefereeSender::integer(0x000173, 2, WHITE, 2, 1250, 565, 15, static_cast<int32_t>(shoot_output)),
        RefereeSender::integer(0x000174, 2, WHITE, 2, 970, 680, 15, static_cast<int32_t>(yaw_angle / deg)),
        RefereeSender::integer(0x000175, 2, WHITE, 2, 780, 565, 15, static_cast<int32_t>(pitch_angle / deg)),
    };
    sender.draw(numbers, 6, RefereeSender::MODIFY);
    draw_car(sender, RefereeSender::MODIFY);
}
