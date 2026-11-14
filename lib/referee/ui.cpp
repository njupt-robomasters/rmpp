#include "ui.hpp"
#include "math/unit.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

UiControl::Graphic UiControl::line(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t end_x, const uint16_t end_y) {
    auto graphic = make_graphic(name, layer, color, width, x, y, LINE);
    graphic.details_d = end_x;
    graphic.details_e = end_y;
    return graphic;
}

UiControl::Graphic UiControl::rectangle(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t end_x, const uint16_t end_y) {
    auto graphic = line(name, layer, color, width, x, y, end_x, end_y);
    graphic.type = RECTANGLE;
    return graphic;
}

UiControl::Graphic UiControl::circle(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t radius) {
    auto graphic = make_graphic(name, layer, color, width, x, y, CIRCLE);
    graphic.details_c = radius;
    return graphic;
}

UiControl::Graphic UiControl::ellipse(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t rx, const uint16_t ry) {
    auto graphic = make_graphic(name, layer, color, width, x, y, ELLIPSE);
    graphic.details_c = rx;
    graphic.details_d = ry;
    return graphic;
}

UiControl::Graphic UiControl::arc(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const float start_angle, const float end_angle, const uint16_t rx, const uint16_t ry) {
    auto graphic = ellipse(name, layer, color, width, x, y, rx, ry);
    graphic.type = ARC;
    set_arc(graphic, start_angle, end_angle);
    return graphic;
}

UiControl::Graphic UiControl::integer(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t font_size, const int32_t value) {
    auto graphic = make_graphic(name, layer, color, width, x, y, INTEGER);
    graphic.details_a = font_size;
    set_number(graphic, value);
    return graphic;
}

UiControl::Graphic UiControl::floating(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t font_size, const float value) {
    auto graphic = integer(name, layer, color, width, x, y, font_size, 0);
    graphic.type = FLOAT;
    set_float(graphic, value);
    return graphic;
}

UiControl::Graphic UiControl::character(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t font_size) {
    auto graphic = make_graphic(name, layer, color, width, x, y, CHARACTER);
    graphic.details_a = font_size;
    return graphic;
}

void UiControl::set_number(Graphic& graphic, const int32_t value) {
    const auto bits = static_cast<uint32_t>(value);
    graphic.details_c = bits & 0x3FF;
    graphic.details_d = (bits >> 10) & 0x7FF;
    graphic.details_e = bits >> 21;
}

void UiControl::set_float(Graphic& graphic, const float value) {
    const double scaled = static_cast<double>(value) * 1000.0;
    int32_t number = 0;
    if (std::isfinite(scaled)) {
        if (scaled >= std::numeric_limits<int32_t>::max())
            number = std::numeric_limits<int32_t>::max();
        else if (scaled <= std::numeric_limits<int32_t>::min())
            number = std::numeric_limits<int32_t>::min();
        else
            number = static_cast<int32_t>(scaled);
    }
    set_number(graphic, number);
}

void UiControl::set_arc(Graphic& graphic, const float start_angle, const float end_angle) {
    // 归一化到协议角度范围，正向整圈保留为360度
    const float angles[] = {start_angle, end_angle};
    uint16_t degrees[2]{};
    for (size_t i = 0; i < 2; i++) {
        if (!std::isfinite(angles[i]))
            continue;
        const float input = angles[i] / deg;
        float angle = std::fmod(input, 360.0f);
        if (angle < 0)
            angle += 360;
        const auto rounded = static_cast<uint16_t>(std::round(angle));
        degrees[i] = rounded == 0 && input > 0 ? 360 : rounded % 360;
    }
    graphic.details_a = degrees[0];
    graphic.details_b = degrees[1];
}

bool UiControl::draw(const Graphic* graphics, const size_t count, const Operation operation) {
    if (!graphics || count == 0 || count > 7)
        return false;
    size_t capacity;
    uint16_t id;
    if (count == 1) {
        capacity = 1;
        id = 0x0101;
    } else if (count == 2) {
        capacity = 2;
        id = 0x0102;
    } else if (count <= 5) {
        capacity = 5;
        id = 0x0103;
    } else {
        capacity = 7;
        id = 0x0104;
    }
    Request request{.content_id = id, .size = static_cast<uint8_t>(capacity * 15)};
    for (size_t i = 0; i < count; i++) {
        if (graphics[i].type == CHARACTER || graphics[i].layer > 9 || graphics[i].color > WHITE)
            return false;
        encode(graphics[i], operation, request.data + i * 15);
    }
    return enqueue(request);
}

bool UiControl::draw_text(const Graphic& graphic, const char* text, const size_t size, const Operation operation) {
    if (size > 30 || (!text && size != 0) || graphic.layer > 9 || graphic.color > WHITE)
        return false;
    auto character = graphic;
    character.type = CHARACTER;
    character.details_b = size;
    Request request{.content_id = 0x0110, .size = 45};
    encode(character, operation, request.data);
    if (size != 0)
        std::memcpy(request.data + 15, text, size);
    return enqueue(request);
}

bool UiControl::delete_layer(const uint8_t layer) {
    if (layer > 9)
        return false;
    return enqueue({.content_id = 0x0100, .size = 2, .data = {1, layer}});
}

bool UiControl::delete_all() {
    return enqueue({.content_id = 0x0100, .size = 2, .data = {2, 0}});
}

void UiControl::clear_queue() {
    head = 0;
    count = 0;
}

void UiControl::update() {
    if (count == 0)
        return;
    const auto sender = robot_id();
    // 只有文档列出的选手端允许自动映射
    if (!((sender >= 1 && sender <= 6) || (sender >= 101 && sender <= 106)))
        return;
    const uint16_t receiver = sender + 0x0100;
    const auto& request = queue[head];
    uint8_t data[111]{};
    data[0] = static_cast<uint8_t>(request.content_id);
    data[1] = static_cast<uint8_t>(request.content_id >> 8);
    data[2] = static_cast<uint8_t>(sender);
    data[3] = static_cast<uint8_t>(sender >> 8);
    data[4] = static_cast<uint8_t>(receiver);
    data[5] = static_cast<uint8_t>(receiver >> 8);
    std::memcpy(data + 6, request.data, request.size);
    if (referee.send(0x0301, data, request.size + 6)) {
        head = (head + 1) % QUEUE_CAPACITY;
        count--;
    }
}

UiControl::Graphic UiControl::make_graphic(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const Type type) {
    return {.name = {static_cast<uint8_t>(name), static_cast<uint8_t>(name >> 8), static_cast<uint8_t>(name >> 16)},
            .operation = NONE,
            .type = type,
            .layer = layer,
            .color = color,
            .details_a = 0,
            .details_b = 0,
            .width = width,
            .x = x,
            .y = y,
            .details_c = 0,
            .details_d = 0,
            .details_e = 0};
}

void UiControl::encode(const Graphic& graphic, const Operation operation, uint8_t* data) {
    std::memcpy(data, graphic.name, 3);
    const uint32_t words[] = {
        static_cast<uint32_t>(operation) | (graphic.type << 3) | (graphic.layer << 6) | (graphic.color << 10) | (graphic.details_a << 14) | (graphic.details_b << 23),
        static_cast<uint32_t>(graphic.width) | (static_cast<uint32_t>(graphic.x) << 10) | (static_cast<uint32_t>(graphic.y) << 21),
        static_cast<uint32_t>(graphic.details_c) | (static_cast<uint32_t>(graphic.details_d) << 10) | (static_cast<uint32_t>(graphic.details_e) << 21)};
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 4; j++)
            data[3 + i * 4 + j] = static_cast<uint8_t>(words[i] >> (j * 8));
}

bool UiControl::enqueue(const Request& request) {
    if (request.content_id == 0x0104 && request.size == 105) {
        const uint32_t name = request.data[0] | (static_cast<uint32_t>(request.data[1]) << 8) | (static_cast<uint32_t>(request.data[2]) << 16);
        if (name == 0x000160 || name == 0x000170) {
            for (size_t i = 0; i < count; i++) {
                auto& queued = queue[(head + i) % QUEUE_CAPACITY];
                const uint32_t queued_name = queued.data[0] | (static_cast<uint32_t>(queued.data[1]) << 8) | (static_cast<uint32_t>(queued.data[2]) << 16);
                if (queued.content_id == request.content_id && queued.size == request.size && queued_name == name) {
                    queued = request;
                    return true;
                }
            }
        }
    }
    if (count == QUEUE_CAPACITY)
        return false;
    queue[(head + count) % QUEUE_CAPACITY] = request;
    count++;
    return true;
}

void RobotUi::init() {
    clear_queue();
    initialized = false;
    enqueue_initial_layout();
    initialized = true;
}

void RobotUi::update() {
    if (!initialized)
        init();

    if (dashboard_timer.elapsed() >= 1.0f / 30.0f) {
        dashboard_timer.restart();
        update_dashboard();
    }

    UiControl::update();
}

void RobotUi::update_dashboard() {
    const auto green = GREEN;
    const auto progress = std::clamp(cap / max_cap, 0.0f, 1.0f);
    const auto speed = std::clamp(std::fabs(wr) / max_wr, 0.0f, 1.0f);
    const auto bullet_ratio = std::clamp(bullet / max_bullet, 0.0f, 1.0f);
    const auto shoot_ratio = std::clamp(std::fabs(shoot) / max_shoot, 0.0f, 1.0f);
    const auto span = [](float ratio) { return static_cast<uint16_t>(1 + ratio * 38); };
    Graphic arcs[7] = {
        arc(0x000160, 1, robot.referee, 10, 960, 540, yaw_ecd - 20 * deg, yaw_ecd + 20 * deg, 85, 85),
        arc(0x000161, 1, PINK, is_hurt ? 10 : 0, 960, 540, -hurt_dir - 20 * deg, -hurt_dir + 20 * deg, 85, 85),
        arc(0x000162, 1, green, 14, 960, 540, 275 * deg, (275 + span(progress)) * deg, 387, 387),
        arc(0x000163, 1, green, 14, 960, 540, (265 - span(speed)) * deg, 265 * deg, 387, 387),
        arc(0x000164, 1, green, 14, 960, 540, 95 * deg, (95 + span(bullet_ratio)) * deg, 387, 387),
        arc(0x000165, 1, green, 14, 960, 540, (85 - span(shoot_ratio)) * deg, 85 * deg, 387, 387),
        circle(0x000166, 1, robot.aim, is_detect ? 21 : 0, 960, 540, 21),
    };
    set_arc(arcs[0], yaw_ecd - 20 * deg, yaw_ecd + 20 * deg);
    set_arc(arcs[1], -hurt_dir - 20 * deg, -hurt_dir + 20 * deg);
    draw(arcs, 7, MODIFY);

    Graphic numbers[6] = {
        integer(0x000170, 2, WHITE, 2, 615, 565, 15, static_cast<int32_t>(cap / pct)),
        integer(0x000171, 2, WHITE, 2, 615, 530, 15, static_cast<int32_t>(wr / rpm)),
        integer(0x000172, 2, WHITE, 2, 1250, 530, 15, static_cast<int32_t>(bullet * (bullet_is_speed ? 10 : 1))),
        integer(0x000173, 2, WHITE, 2, 1250, 565, 15, static_cast<int32_t>(shoot)),
        integer(0x000174, 2, WHITE, 2, 970, 680, 15, static_cast<int32_t>(yaw / deg)),
        integer(0x000175, 2, WHITE, 2, 780, 565, 15, static_cast<int32_t>(pitch / deg)),
    };
    draw(numbers, 6, MODIFY);
}

void RobotUi::enqueue_initial_layout() {
    const auto white = WHITE;
    const auto green = GREEN;

    Graphic cross[4] = {
        line(0x000100, 0, white, 2, 960, 660, 960, 890),
        line(0x000101, 0, white, 2, 840, 540, 610, 540),
        line(0x000102, 0, white, 2, 960, 420, 960, 190),
        line(0x000103, 0, white, 2, 1080, 540, 1310, 540),
    };
    draw(cross, 4, ADD);

    Graphic limits[6]{};
    const uint16_t angles[] = {315, 267, 223, 135, 87, 43};
    const uint16_t ends[] = {317, 273, 225, 137, 93, 45};
    for (size_t i = 0; i < 6; i++)
        limits[i] = arc(0x000110 + i, 0, white, 18, 960, 540, angles[i] * deg, ends[i] * deg, 387, 387);
    draw(limits, 6, ADD);

    const Graphic labels[] = {
        character(0x000120, 0, white, 2, 660, 860, 20),
        character(0x000130, 0, white, 2, 660, 240, 20),
        character(0x000140, 0, white, 2, 1200, 240, 20),
        character(0x000150, 0, white, 2, 1200, 860, 20),
    };
    draw_text(labels[0], "CAP", 3, ADD);
    draw_text(labels[1], "RPM", 3, ADD);
    draw_text(labels[2], "BULLET", 6, ADD);
    draw_text(labels[3], "SHOOT", 5, ADD);

    Graphic dynamic_arcs[7] = {
        arc(0x000160, 1, green, 10, 960, 540, 0 * deg, 360 * deg, 85, 85),
        arc(0x000161, 1, PINK, 0, 960, 540, 0 * deg, 360 * deg, 85, 85),
        arc(0x000162, 1, green, 14, 960, 540, 275 * deg, 313 * deg, 387, 387),
        arc(0x000163, 1, green, 14, 960, 540, 227 * deg, 265 * deg, 387, 387),
        arc(0x000164, 1, green, 14, 960, 540, 95 * deg, 133 * deg, 387, 387),
        arc(0x000165, 1, green, 14, 960, 540, 47 * deg, 85 * deg, 387, 387),
        circle(0x000166, 1, green, 2, 960, 540, 21),
    };
    draw(dynamic_arcs, 7, ADD);

    Graphic numbers[6] = {
        integer(0x000170, 2, white, 2, 615, 565, 15, 0),
        integer(0x000171, 2, white, 2, 615, 530, 15, 0),
        integer(0x000172, 2, white, 2, 1250, 530, 15, 0),
        integer(0x000173, 2, white, 2, 1250, 565, 15, 0),
        integer(0x000174, 2, white, 2, 970, 680, 15, 0),
        integer(0x000175, 2, white, 2, 780, 565, 15, 0),
    };
    draw(numbers, 6, ADD);

    Graphic wheel[7] = {
        arc(0x003000, 3, green, 2, 290, 551, 90 * deg, 217 * deg, 30, 25),
        arc(0x003001, 3, green, 2, 142, 551, 90 * deg, 217 * deg, 30, 25),
        arc(0x003002, 3, green, 2, 100, 546, 90 * deg, 270 * deg, 30, 25),
        arc(0x003003, 3, green, 2, 250, 546, 90 * deg, 270 * deg, 30, 25),
        line(0x003004, 3, green, 2, 280, 551, 320, 571),
        line(0x003005, 3, green, 2, 132, 551, 172, 571),
        line(0x003006, 3, green, 2, 70, 546, 130, 571),
    };
    draw(wheel, 7, ADD);

    Graphic modules[7] = {
        line(0x003100, 3, green, 2, 220, 546, 280, 571),
        line(0x003101, 3, green, 2, 163, 588, 223, 608),
        line(0x003102, 3, green, 2, 56, 573, 331, 623),
        line(0x003103, 3, green, 2, 116, 639, 241, 689),
        line(0x003104, 3, green, 2, 243, 648, 323, 678),
        line(0x003105, 3, green, 2, 257, 664, 269, 676),
        line(0x003106, 3, green, 2, 257, 650, 269, 662),
    };
    draw(modules, 7, ADD);

    Graphic modules_tail[4] = {
        line(0x003200, 3, green, 2, 271, 664, 283, 676),
        line(0x003201, 3, green, 2, 271, 650, 283, 662),
        line(0x003202, 3, green, 2, 156, 625, 231, 637),
        line(0x003203, 3, green, 2, 298, 680, 323, 692),
    };
    draw(modules_tail, 4, ADD);
}
