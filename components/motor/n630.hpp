#pragma once

#include "unit/include_me.hpp"

class N630 {
public:
    static constexpr uint8_t POLE_PAIR = 7; // 电机极对数

    enum brake_mode_e {
        NO_BRAKE,
        ALWAYS_BRAKE,
        AUTO_BRAKE
    };

    // CAN通信参数
    const uint8_t can_port;
    const uint32_t n630_id;

    // 使能标志
    bool is_enable = false;

    // 刹车模式
    brake_mode_e brake_mode = NO_BRAKE;

    // status 1
    struct {
        UnitFloat<rpm> ref, measure;
    } speed;

    UnitFloat<A> current;
    UnitFloat<pct> duty;

    // status 4
    UnitFloat<C> temperate_mos;
    UnitFloat<C> temperate_motor;
    UnitFloat<A> current_in;
    UnitFloat<deg> pid_pos;

    // status 5
    UnitFloat<> tachometer; // todo: 转速计的是单位是什么
    UnitFloat<V> voltage_in;

    N630(uint8_t can_port, uint32_t n630_id);

    // 设置使能/失能
    void SetEnable(bool is_enable);

    // 设置刹车模式
    void SetBrakeMode(brake_mode_e brake_mode);

    // 设置速度
    void SetSpeed(const UnitFloat<>& speed);

    // 需要在循环中调用
    void OnLoop() const;

private:
    enum command_e {
        CAN_PACKET_SET_DUTY                  = 0,
        CAN_PACKET_SET_CURRENT               = 1,
        CAN_PACKET_SET_CURRENT_BRAKE         = 2,
        CAN_PACKET_SET_RPM                   = 3,
        CAN_PACKET_SET_POS                   = 4,
        CAN_PACKET_SET_CURRENT_REL           = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL     = 11,
        CAN_PACKET_SET_CURRENT_HANDBRAKE     = 12,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13
    };

    enum status_e {
        CAN_PACKET_STATUS   = 9,
        CAN_PACKET_STATUS_2 = 14,
        CAN_PACKET_STATUS_3 = 15,
        CAN_PACKET_STATUS_4 = 16,
        CAN_PACKET_STATUS_5 = 27,
        CAN_PACKET_STATUS_6 = 58
    };

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    // 构造CAN报文
    static void makeCANData(uint8_t data[4], float value, float scale);

    // 发送erpm
    void sendERPM(float erpm) const;

    // 发送刹车电流
    void sendCurrentBrake(float current) const;
};
