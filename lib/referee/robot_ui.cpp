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

    // 图层 3 整车示意图：轮子
    const Figure wheel[] = {
        RefereeSender::arc(0x003000, 3, GREEN, 2, 290, 551, 90 * deg, 217 * deg, 30, 25),
        RefereeSender::arc(0x003001, 3, GREEN, 2, 142, 551, 90 * deg, 217 * deg, 30, 25),
        RefereeSender::arc(0x003002, 3, GREEN, 2, 100, 546, 90 * deg, 270 * deg, 30, 25),
        RefereeSender::arc(0x003003, 3, GREEN, 2, 250, 546, 90 * deg, 270 * deg, 30, 25),
        RefereeSender::line(0x003004, 3, GREEN, 2, 280, 551, 320, 571),
        RefereeSender::line(0x003005, 3, GREEN, 2, 132, 551, 172, 571),
        RefereeSender::line(0x003006, 3, GREEN, 2, 70, 546, 130, 571),
    };
    sender.draw(wheel, 7, RefereeSender::ADD);

    // 图层 3 整车示意图：车身与云台
    const Figure modules[] = {
        RefereeSender::line(0x003100, 3, GREEN, 2, 220, 546, 280, 571),
        RefereeSender::line(0x003101, 3, GREEN, 2, 163, 588, 223, 608),
        RefereeSender::line(0x003102, 3, GREEN, 2, 56, 573, 331, 623),
        RefereeSender::line(0x003103, 3, GREEN, 2, 116, 639, 241, 689),
        RefereeSender::line(0x003104, 3, GREEN, 2, 243, 648, 323, 678),
        RefereeSender::line(0x003105, 3, GREEN, 2, 257, 664, 269, 676),
        RefereeSender::line(0x003106, 3, GREEN, 2, 257, 650, 269, 662),
    };
    sender.draw(modules, 7, RefereeSender::ADD);

    const Figure modules_tail[] = {
        RefereeSender::line(0x003200, 3, GREEN, 2, 271, 664, 283, 676),
        RefereeSender::line(0x003201, 3, GREEN, 2, 271, 650, 283, 662),
        RefereeSender::line(0x003202, 3, GREEN, 2, 156, 625, 231, 637),
        RefereeSender::line(0x003203, 3, GREEN, 2, 298, 680, 323, 692),
    };
    sender.draw(modules_tail, 4, RefereeSender::ADD);
}

void RobotUi::update(RefereeSender& sender, const Display& display) {
    // 提示条：满量程比例映射为 1~39 度弧长
    const auto span = [](const float ratio) { return static_cast<uint16_t>(1 + std::clamp(ratio, 0.0f, 1.0f) * 38); };
    const auto cap = span(display.cap / config.cap_max);
    const auto wr = span(std::fabs(display.wr) / config.wr_max);
    const auto bullet = span(display.bullet / config.bullet_max);
    const auto shoot = span(std::fabs(display.shoot) / config.shoot_max);

    // 图层 1：云台朝向 ±20 度弧（裁判在线色）、受击方向弧、四角提示条、识别时准星圆（自瞄在线色）
    const Figure arcs[] = {
        RefereeSender::arc(0x000160, 1, display.referee, 10, 960, 540, display.yaw_ecd - 20 * deg, display.yaw_ecd + 20 * deg, 85, 85),
        RefereeSender::arc(0x000161, 1, PINK, display.is_hurt ? 10 : 0, 960, 540, -display.hurt_dir - 20 * deg, -display.hurt_dir + 20 * deg, 85, 85),
        RefereeSender::arc(0x000162, 1, GREEN, 14, 960, 540, 275 * deg, (275 + cap) * deg, 387, 387),
        RefereeSender::arc(0x000163, 1, GREEN, 14, 960, 540, (265 - wr) * deg, 265 * deg, 387, 387),
        RefereeSender::arc(0x000164, 1, GREEN, 14, 960, 540, 95 * deg, (95 + bullet) * deg, 387, 387),
        RefereeSender::arc(0x000165, 1, GREEN, 14, 960, 540, (85 - shoot) * deg, 85 * deg, 387, 387),
        RefereeSender::circle(0x000166, 1, display.aim, display.is_detect ? 21 : 0, 960, 540, 21),
    };
    sender.draw(arcs, 7, RefereeSender::MODIFY);

    // 图层 2：能量 %、转速 rpm、弹频 Hz、拨弹电流 A、yaw 度、pitch 度
    const Figure numbers[] = {
        RefereeSender::integer(0x000170, 2, WHITE, 2, 615, 565, 15, static_cast<int32_t>(display.cap / pct)),
        RefereeSender::integer(0x000171, 2, WHITE, 2, 615, 530, 15, static_cast<int32_t>(display.wr / rpm)),
        RefereeSender::integer(0x000172, 2, WHITE, 2, 1250, 530, 15, static_cast<int32_t>(display.bullet)),
        RefereeSender::integer(0x000173, 2, WHITE, 2, 1250, 565, 15, static_cast<int32_t>(display.shoot)),
        RefereeSender::integer(0x000174, 2, WHITE, 2, 970, 680, 15, static_cast<int32_t>(display.yaw / deg)),
        RefereeSender::integer(0x000175, 2, WHITE, 2, 780, 565, 15, static_cast<int32_t>(display.pitch / deg)),
    };
    sender.draw(numbers, 6, RefereeSender::MODIFY);
}
