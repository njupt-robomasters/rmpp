#pragma once

#include "math/unit.hpp"
#include "motor.hpp"

class Vesc : public Motor {
  public:
    static constexpr Config DEFAULT_CONFIG{
        .physical = {.kt = 1},
        .install = {.reduction = 1},
    };

    struct VescConfig {
        uint8_t pole_pair = 0;         // 电机极对数；零采用默认值 7
        float auto_release_speed = -1; // 低速自动释放阈值，rad/s；负值采用默认值，零禁用
    } const vesc_config;

    // 母线电压电流
    struct Bus {
        float voltage{}; // 母线电压
        float current{}; // 母线电流
    } bus;

    float duty{}; // 占空比

    explicit Vesc(const Config& config);
    Vesc(const Config& config, VescConfig vesc_config);

    // 重写父类的闭环操作
    void update_control() override;

    bool send_can_cmd();

  private:
    enum Command {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT = 1,
        CAN_PACKET_SET_CURRENT_BRAKE = 2,
        CAN_PACKET_SET_RPM = 3,
        CAN_PACKET_SET_POS = 4,
        CAN_PACKET_SET_CURRENT_REL = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
        CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13
    };

    enum Status {
        CAN_PACKET_STATUS = 9,
        CAN_PACKET_STATUS_2 = 14,
        CAN_PACKET_STATUS_3 = 15,
        CAN_PACKET_STATUS_4 = 16,
        CAN_PACKET_STATUS_5 = 27,
        CAN_PACKET_STATUS_6 = 58
    };

    // CAN接收回调
    void receive(const CanFrame& frame) override;

    // 构造CAN报文
    static void make_can_data(uint8_t data[4], float value, float scale);

    // 发送erpm
    bool send_erpm(float erpm) const;

    // 发送刹车电流
    bool send_current_brake(float current) const;
};
