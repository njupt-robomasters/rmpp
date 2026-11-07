#pragma once

#include <cstdint>

#include "motor.hpp"

class Vesc : public Motor {
  public:
    static constexpr float MAX_CURRENT = 20;
    static constexpr float KT = 1;
    static constexpr float REDUCTION = 7;

    float temp_fet = 0;
    float temp_motor = 0;
    float current_in = 0;
    float voltage_in = 0;
    float duty_cycle = 0;
    float amp_hours = 0;
    float amp_hours_charged = 0;
    float watt_hours = 0;
    float watt_hours_charged = 0;
    float pid_pos = 0;
    float tachometer = 0;
    float adc1 = 0;
    float adc2 = 0;
    float adc3 = 0;
    float ppm = 0;

    explicit Vesc(const Motor::Config& config);

    void set_current(float current) override;
    void set_torque(float torque) override;
    void set_duty(float duty);
    void set_current_brake(float current);
    // 输出轴速度，rad/s；VESC 按 ERPM 闭环，reduction 含极对数
    void set_speed(float speed);
    void set_position(float position);
    void set_current_relative(float current);
    void set_current_brake_relative(float current);
    void set_handbrake(float current);
    void set_handbrake_relative(float current);
    void set_current_off_delay(float current, float delay);
    void set_current_relative_off_delay(float current, float delay);
    void send() const;

  private:
    enum Command : uint8_t {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT = 1,
        CAN_PACKET_SET_CURRENT_BRAKE = 2,
        CAN_PACKET_SET_RPM = 3,
        CAN_PACKET_SET_POS = 4,
        CAN_PACKET_SET_CURRENT_REL = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
        CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    };

    enum Status : uint8_t {
        CAN_PACKET_STATUS = 9,
        CAN_PACKET_STATUS_2 = 14,
        CAN_PACKET_STATUS_3 = 15,
        CAN_PACKET_STATUS_4 = 16,
        CAN_PACKET_STATUS_5 = 27,
        CAN_PACKET_STATUS_6 = 58,
    };

    Command command_ = CAN_PACKET_SET_CURRENT;
    uint8_t command_data_[8]{};
    uint8_t command_size_ = 4;

    void can_callback(const CanFrame& frame);
    void set_int32(Command command, int32_t value);
    void set_int16(Command command, int32_t first, int16_t second);

    static int16_t encode16(float value, float scale);
    static int32_t encode32(float value, float scale);
    static int16_t read16(const uint8_t* data);
    static int32_t read32(const uint8_t* data);
};
