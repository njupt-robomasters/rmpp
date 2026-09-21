#pragma once

#include "math/conversions.hpp"
#include "motor.hpp"

using math::rpm;

class Vesc : public Motor {
  public:
    struct VescConfig {
        uint8_t pole_pair = 7;                // 极对数
        float auto_release_speed = 120 * rpm; // 自动释放电机的转速，避免持续的高频注入声音（0表示禁用）
    } vesc_config;

    // 母线电压电流
    struct Bus {
        float voltage{}; // 母线电压
        float current{}; // 母线电流
    } bus;

    float duty{}; // 占空比

    Vesc(const Config& config, const VescConfig& vesc_config);

    // 重写父类的闭环操作
    void on_loop() override;

    void send_can_cmd();

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
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    // 构造CAN报文
    static void make_can_data(uint8_t data[4], float value, float scale);

    // 发送erpm
    void send_erpm(float erpm) const;

    // 发送刹车电流
    void send_current_brake(float current) const;
};