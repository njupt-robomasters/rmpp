#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
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
    bool is_enable = false;    // 底盘使能，下板直接透传
    bool allow_rescue = false; // 批准自救，倒地时下板据此启动
    bool trigger_jump = false; // 跳跃触发

    // 运动
    float vx = 0;  // 前向速度，m/s
    float wr = 0;  // 航向角速度目标，rad/s；小陀螺为手动角速度，跟随为云台 yaw 目标速度前馈
    float yaw = 0; // 底盘航向误差，rad；云台相对底盘角加车头跟随偏角，小陀螺发 0

    float leg_length = 0; // 腿长，m
    float power_ref = 0;  // 底盘功率目标，W；INFINITY 表示不限功率

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
    static constexpr size_t MOTOR_COUNT = 6; // 左前、左后、右后、右前、左轮、右轮

    // 下板电机状态，按离线、达妙错误、过热、正常的顺序判断
    enum class MotorState : uint8_t {
        NORMAL,   // 正常
        OVERHEAT, // 过热
        ERROR,    // 达妙电机错误码
        OFFLINE,  // 反馈超时
    };

    // DOWN_ID，4 字节；字段与成员同序，从低位到高位连续排列
    struct __attribute__((packed)) Raw {
        bool is_fallen : 1;
        int16_t wr : 15; // 1e-3 rad/s/LSB

        // 电机状态，见 MotorState
        MotorState left_front : 2;
        MotorState left_back : 2;
        MotorState right_back : 2;
        MotorState right_front : 2;
        MotorState left_wheel : 2;
        MotorState right_wheel : 2;

        uint8_t reserved : 4; // 保留，发送时为 0
    };

    bool is_fallen = true; // 底盘倒地；下板失能或断联时保持 true，上板据此要求先批准自救
    float wr = 0;          // 实测航向角速度，rad/s

    MotorState motor[MOTOR_COUNT]{}; // 左前、左后、右后、右前、左轮、右轮

    void pack(CanFrame& frame) const {
        const Raw raw{
            .is_fallen = is_fallen,
            .wr = static_cast<int16_t>(std::lround(std::clamp(wr, -16.384f, 16.383f) * 1000)),
            .left_front = motor[0],
            .left_back = motor[1],
            .right_back = motor[2],
            .right_front = motor[3],
            .left_wheel = motor[4],
            .right_wheel = motor[5],
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
        motor[0] = raw.left_front;
        motor[1] = raw.left_back;
        motor[2] = raw.right_back;
        motor[3] = raw.right_front;
        motor[4] = raw.left_wheel;
        motor[5] = raw.right_wheel;
        return true;
    }
};

template <typename TxFrame, typename RxFrame>
class LegProtocol : public TxFrame {
  public:
    RxFrame received;
    bool is_connect = false;

    explicit LegProtocol(const Can can)
        : can(can) {
        this->can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
    }

    void update() {
        if (callback_timer.elapsed() > LEG_TIMEOUT) {
            is_connect = false;
            received = {};
        }
    }

    void send() {
        CanFrame frame;
        this->pack(frame);
        can.send(frame);
    }

  private:
    const Can can;
    Timer callback_timer;

    void can_callback(const CanFrame& frame) {
        if (!received.unpack(frame))
            return;

        callback_timer.restart();
        is_connect = true;
    }
};

using LegUpProtocol = LegProtocol<LegUpFrame, LegDownFrame>;
using LegDownProtocol = LegProtocol<LegDownFrame, LegUpFrame>;
