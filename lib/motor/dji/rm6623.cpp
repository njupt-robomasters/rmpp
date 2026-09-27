#include "rm6623.hpp"

RM6623::RM6623(const Config& config, Protocol protocol)
    : DjiMotor(apply_defaults(config, DEFAULT_CONFIG),
               {.feedback_scale = protocol.feedback_scale ? protocol.feedback_scale : DEFAULT_PROTOCOL.feedback_scale,
                .command_scale = protocol.command_scale ? protocol.command_scale : DEFAULT_PROTOCOL.command_scale}) {
    subscribe();
}

void RM6623::parse_feedback(const uint8_t data[8]) {
    // RM6623 没有速度反馈，电流位于第 2、3 字节
    accept_dji_feedback((data[0] << 8) | data[1], static_cast<int16_t>((data[2] << 8) | data[3]));
}

bool RM6623::send_calibrate_cmd() const {
    return config.protocol.can.send({.id = 0x3f0, .size = 8, .data = {'c'}});
}
