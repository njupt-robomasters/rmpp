#pragma once

#include "bsp/can.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

// CAN1 板间协议，按发送板命名；位域按 STM32 GCC 小端布局，主机 MinGW 测试需加 -mno-ms-bitfields
inline constexpr uint32_t UP_ID = 0x02, DOWN_ID = 0x03;
inline constexpr float LEG_TIMEOUT = 0.1f; // 状态帧超时判断下板断联，s

struct LegUpFrame {
    // UP_ID，8 字节；字段与成员同序，从低位到高位连续排列
    struct __attribute__((packed)) Raw {
        // 开关，0/1
        bool is_enable : 1;
        bool allow_rescue : 1;
        bool trigger_jump : 1;

        // 运动
        int16_t vx : 13;  // 1e-3 m/s/LSB
        int16_t wr : 15;  // 1e-3 rad/s/LSB
        int16_t yaw : 13; // 1e-3 rad/LSB

        uint16_t leg_length : 9; // 1 mm/LSB
        uint16_t power_ref : 11; // 0.25 W/LSB；2047 表示不限功率
    };

    // 开关
    bool is_enable = false;    // 底盘使能
    bool allow_rescue = false; // 自救授权
    bool trigger_jump = false; // 跳跃触发

    // 运动
    float vx = 0;  // 前向速度，m/s
    float wr = 0;  // 手动角速度，rad/s
    float yaw = 0; // 云台相对底盘航向角，rad

    float leg_length = 0; // 腿长，m
    float power_ref = 0;  // 底盘功率目标，W；无穷大表示不限功率

    void pack(CanFrame& frame) const {
        const Raw raw{
            .is_enable = is_enable,
            .allow_rescue = allow_rescue,
            .trigger_jump = trigger_jump,
            .vx = static_cast<int16_t>(std::lround(std::clamp(vx, -4.096f, 4.095f) * 1000)),
            .wr = static_cast<int16_t>(std::lround(std::clamp(wr, -16.384f, 16.383f) * 1000)),
            .yaw = static_cast<int16_t>(std::lround(std::clamp(yaw, -4.096f, 4.095f) * 1000)),
            .leg_length = static_cast<uint16_t>(std::lround(std::clamp(leg_length, 0.0f, 0.511f) * 1000)),
            .power_ref = std::isinf(power_ref) ? static_cast<uint16_t>(2047) : static_cast<uint16_t>(std::clamp(std::lround(power_ref * 4), 0l, 2046l)),
        };

        frame.id = UP_ID;
        frame.size = sizeof(raw);
        std::memcpy(frame.data, &raw, sizeof(raw));
    }

    bool unpack(const CanFrame& frame) {
        if (frame.id != UP_ID || frame.size != sizeof(Raw))
            return false;
        Raw raw;
        std::memcpy(&raw, frame.data, sizeof(raw));

        is_enable = raw.is_enable;
        allow_rescue = raw.allow_rescue;
        trigger_jump = raw.trigger_jump;
        vx = raw.vx / 1000.0f;
        wr = raw.wr / 1000.0f;
        yaw = raw.yaw / 1000.0f;
        leg_length = raw.leg_length / 1000.0f;
        power_ref = raw.power_ref == 2047 ? INFINITY : raw.power_ref * 0.25f;
        return true;
    }
};

struct LegDownFrame {
    // DOWN_ID，3 字节；字段与成员同序，从低位到高位连续排列
    struct __attribute__((packed)) Raw {
        bool is_fallen : 1;
        int16_t wr : 16;      // 1e-3 rad/s/LSB
        uint8_t reserved : 7; // 保留，发送时为 0
    };

    bool is_fallen = false; // 底盘倒地，站起完成或失能时清除
    float wr = 0;           // 实测航向角速度，rad/s

    void pack(CanFrame& frame) const {
        const Raw raw{
            .is_fallen = is_fallen,
            .wr = static_cast<int16_t>(std::lround(std::clamp(wr, -32.768f, 32.767f) * 1000)),
            .reserved = 0,
        };

        frame.id = DOWN_ID;
        frame.size = sizeof(raw);
        std::memcpy(frame.data, &raw, sizeof(raw));
    }

    bool unpack(const CanFrame& frame) {
        if (frame.id != DOWN_ID || frame.size != sizeof(Raw))
            return false;
        Raw raw;
        std::memcpy(&raw, frame.data, sizeof(raw));

        is_fallen = raw.is_fallen;
        wr = raw.wr / 1000.0f;
        return true;
    }
};
