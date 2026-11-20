#include "referee_sender.hpp"
#include "crc.hpp"
#include "math/unit.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

RefereeSender::RefereeSender(const Config& config)
    : config(config) {}

void RefereeSender::set_robot_id(const uint16_t robot_id) {
    this->robot_id = robot_id;
}

Figure RefereeSender::line(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t end_x, const uint16_t end_y) {
    auto figure = make_figure(name, layer, color, width, x, y, LINE);
    figure.details_d = end_x;
    figure.details_e = end_y;
    return figure;
}

Figure RefereeSender::rectangle(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t end_x, const uint16_t end_y) {
    auto figure = line(name, layer, color, width, x, y, end_x, end_y);
    figure.type = RECTANGLE;
    return figure;
}

Figure RefereeSender::circle(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t radius) {
    auto figure = make_figure(name, layer, color, width, x, y, CIRCLE);
    figure.details_c = radius;
    return figure;
}

Figure RefereeSender::ellipse(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t rx, const uint16_t ry) {
    auto figure = make_figure(name, layer, color, width, x, y, ELLIPSE);
    figure.details_d = rx;
    figure.details_e = ry;
    return figure;
}

Figure RefereeSender::arc(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const float start_angle, const float end_angle, const uint16_t rx, const uint16_t ry) {
    auto figure = ellipse(name, layer, color, width, x, y, rx, ry);
    figure.type = ARC;
    write_arc(figure, start_angle, end_angle);
    return figure;
}

Figure RefereeSender::integer(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t font_size, const int32_t value) {
    auto figure = make_figure(name, layer, color, width, x, y, INTEGER);
    figure.details_a = font_size;
    write_number(figure, value);
    return figure;
}

Figure RefereeSender::floating(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t font_size, const float value) {
    auto figure = make_figure(name, layer, color, width, x, y, FLOAT);
    figure.details_a = font_size;
    write_float(figure, value);
    return figure;
}

Figure RefereeSender::character(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const uint16_t font_size) {
    auto figure = make_figure(name, layer, color, width, x, y, CHARACTER);
    figure.details_a = font_size;
    return figure;
}

bool RefereeSender::draw(const Figure* figures, const size_t count, const Operation operation) {
    if (!figures || count == 0 || count > 7)
        return false;

    // 子内容按图形数取 1、2、5、7 个的档位，空位补 0
    Request request{.cmd_id = 0x0301};
    if (count == 1) {
        request.content_id = 0x0101;
        request.size = 15;
    } else if (count == 2) {
        request.content_id = 0x0102;
        request.size = 30;
    } else if (count <= 5) {
        request.content_id = 0x0103;
        request.size = 75;
    } else {
        request.content_id = 0x0104;
        request.size = 105;
    }
    for (size_t i = 0; i < count; i++) {
        if (figures[i].type == CHARACTER || figures[i].layer > 9 || figures[i].color > WHITE)
            return false;
        auto figure = figures[i];
        figure.operation = operation;
        std::memcpy(request.content + i * sizeof(Figure), &figure, sizeof(Figure));
    }
    return push(request);
}

bool RefereeSender::draw_text(const Figure& figure, const char* text, const size_t size, const Operation operation) {
    if (size > 30 || (!text && size != 0) || figure.layer > 9 || figure.color > WHITE)
        return false;

    // 字符配置 15 B 后接 30 B 字符
    Request request{.cmd_id = 0x0301, .content_id = 0x0110, .size = 45};
    auto character = figure;
    character.operation = operation;
    character.type = CHARACTER;
    character.details_b = size;
    std::memcpy(request.content, &character, sizeof(Figure));
    if (size != 0)
        std::memcpy(request.content + sizeof(Figure), text, size);
    return push(request);
}

bool RefereeSender::delete_layer(const uint8_t layer) {
    if (layer > 9)
        return false;
    return push({.cmd_id = 0x0301, .content_id = 0x0100, .size = 2, .content = {1, layer}});
}

bool RefereeSender::delete_all() {
    return push({.cmd_id = 0x0301, .content_id = 0x0100, .size = 2, .content = {2, 0}});
}

void RefereeSender::clear_ui() {
    // 逐个取出，非 UI 请求按原顺序放回
    const size_t count = queue.size;
    for (size_t i = 0; i < count; i++) {
        Request request;
        queue.pop(&request, 1);
        if (request.cmd_id != 0x0301 || request.content_id < 0x0100 || request.content_id > 0x0110)
            queue.push(request);
    }
}

bool RefereeSender::send_sentry_command(const SentryCommand& command) {
    Request request{.cmd_id = 0x0301, .content_id = 0x0120, .receiver_id = 0x8080, .size = sizeof(command)};
    std::memcpy(request.content, &command, sizeof(command));
    return push(request);
}

bool RefereeSender::send_custom_info(const uint16_t receiver_id, const char16_t* text, const size_t size) {
    if (receiver_id == 0 || size > 15 || (!text && size != 0))
        return false;

    // 30 B 小端 UTF-16，不足补 0
    Request request{.cmd_id = 0x0308, .receiver_id = receiver_id, .size = 30};
    for (size_t i = 0; i < size; i++)
        write_u16(request.content + i * 2, static_cast<uint16_t>(text[i]));
    return push(request);
}

void RefereeSender::send() {
    // 组帧：上一帧发完且本机 ID 已知时取队首请求
    if (frame_size == 0 && queue.size != 0 && robot_id != 0) {
        const auto& request = queue[0];
        const bool is_interaction = request.cmd_id == 0x0301;
        const auto index = robot_id % 100; // 红方 1~11，蓝方 101~111
        const bool has_client = index == 1 || (index >= 3 && index <= 6);

        if (request.receiver_id == 0 && !has_client) {
            // 发往本机选手端的 UI：工程、哨兵等无选手端时丢弃
            queue.pop(1);
        } else if (!is_interaction || interaction_timer.elapsed() >= INTERACTION_PERIOD) {
            // 载荷：0x0301 为子内容 ID、发送者、接收者、内容；0x0308 为发送者、接收者、内容
            const uint16_t receiver_id = request.receiver_id != 0 ? request.receiver_id : robot_id + 0x0100;
            uint8_t* payload = frame + 7;
            size_t size = 0;
            if (is_interaction) {
                write_u16(payload, request.content_id);
                size = 2;
            }
            write_u16(payload + size, robot_id);
            write_u16(payload + size + 2, receiver_id);
            size += 4;
            std::memcpy(payload + size, request.content, request.size);
            size += request.size;

            // 帧头 0xA5、载荷长度、包序号、CRC8，命令码，整帧 CRC16
            frame[0] = 0xA5;
            write_u16(frame + 1, static_cast<uint16_t>(size));
            frame[3] = seq++;
            frame[4] = CRC8::check_sum(frame, 4);
            write_u16(frame + 5, request.cmd_id);
            write_u16(frame + 7 + size, CRC16::check_sum(frame, 7 + size));
            frame_size = size + 9;
            frame_pos = 0;

            if (is_interaction)
                interaction_timer.restart();
            queue.pop(1);
        }
    }

    // 发送：CAN 有空位时发出至多 8 B；忙时保留，下次续发
    if (frame_size == 0 || !config.can.can_send())
        return;
    CanFrame can_frame{.id = config.tx_id, .size = static_cast<uint8_t>(std::min<size_t>(8, frame_size - frame_pos))};
    std::memcpy(can_frame.data, frame + frame_pos, can_frame.size);
    config.can.send(can_frame);
    frame_pos += can_frame.size;
    if (frame_pos == frame_size)
        frame_size = 0;
}

Figure RefereeSender::make_figure(const uint32_t name, const uint8_t layer, const Color color, const uint16_t width, const uint16_t x, const uint16_t y, const Type type) {
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

void RefereeSender::write_arc(Figure& figure, const float start_angle, const float end_angle) {
    // 归一化到 [0, 360) 度，正向整圈保留为 360；非有限值为 0
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
    figure.details_a = degrees[0];
    figure.details_b = degrees[1];
}

void RefereeSender::write_number(Figure& figure, const int32_t value) {
    // 32 位补码拆入 details_c/d/e
    const auto bits = static_cast<uint32_t>(value);
    figure.details_c = bits & 0x3FF;
    figure.details_d = (bits >> 10) & 0x7FF;
    figure.details_e = bits >> 21;
}

void RefereeSender::write_float(Figure& figure, const float value) {
    // 放大 1000 倍四舍五入，超出 int32 时饱和，非有限值为 0
    const double scaled = static_cast<double>(value) * 1000.0;
    int32_t number = 0;
    if (std::isfinite(scaled)) {
        if (scaled >= std::numeric_limits<int32_t>::max())
            number = std::numeric_limits<int32_t>::max();
        else if (scaled <= std::numeric_limits<int32_t>::min())
            number = std::numeric_limits<int32_t>::min();
        else
            number = static_cast<int32_t>(std::round(scaled));
    }
    write_number(figure, number);
}

bool RefereeSender::push(const Request& request) {
    // 仪表刷新只保留最新：同子内容、首图形同名且同为 MODIFY 的排队请求原位替换
    const bool is_modify = request.cmd_id == 0x0301 && request.content_id >= 0x0101 && request.content_id <= 0x0104 && (request.content[3] & 0x07) == MODIFY;
    if (is_modify) {
        for (size_t i = 0; i < queue.size; i++) {
            auto& queued = queue[i];
            if (queued.cmd_id == request.cmd_id && queued.content_id == request.content_id && (queued.content[3] & 0x07) == MODIFY && std::memcmp(queued.content, request.content, 3) == 0) {
                queued = request;
                return true;
            }
        }
    }
    return queue.push(request);
}

void RefereeSender::write_u16(uint8_t* data, const uint16_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}
