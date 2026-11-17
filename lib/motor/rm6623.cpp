#include "rm6623.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"

RM6623::RM6623(const Config& config)
    : Motor(config, MAX_CURRENT, MAX_TORQUE, REDUCTION) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void RM6623::write_can_frame(CanFrame& frame) const {
    // 计算can偏移量
    uint32_t first_id;
    if (frame.id == 0x1ff)
        first_id = 0x205;
    else if (frame.id == 0x2ff)
        first_id = 0x209;
    else
        return;
    if (frame.size != 8 || config.rx_id < first_id || config.rx_id >= first_id + 4)
        return;
    const uint8_t offset = static_cast<uint8_t>(config.rx_id - first_id) * 2;

    // 计算can命令
    float current = 0;
    if (is_connect && is_enable)
        current = config.is_invert ? -current_ref : current_ref;
    const int16_t command = static_cast<int16_t>(current / MAX_CURRENT * 5000.0f);

    // 构造can报文
    frame.data[offset] = static_cast<uint8_t>(command >> 8);
    frame.data[offset + 1] = static_cast<uint8_t>(command);
}

void RM6623::send_calibrate_cmd() const {
    config.can.send({.id = 0x3f0, .size = 8, .data = {'c'}});
}

void RM6623::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != config.rx_id || frame.size != 8)
        return;

    // 解析can报文
    const auto* data = frame.data;
    // 解析电流和位置反馈，电流位于第 2、3 字节
    const float raw_current = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]) / 5000.0f * MAX_CURRENT;
    const float raw_angle = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]) / 8192.0f * rev;

    // 相邻两帧位置差分得到转速
    const float dt = speed_timer.restart();
    const float raw_speed = has_angle && dt > 0 ? static_cast<float>(Angle(raw_angle - angle_last)) / dt : 0;
    angle_last = raw_angle;
    has_angle = true;

    // 调用上级回调
    Motor::can_callback({.current = raw_current, .speed = raw_speed, .angle = raw_angle});
}
