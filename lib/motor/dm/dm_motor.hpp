#pragma once

#include "motor/motor.hpp"

class DmMotor : public Motor {
  public:
    struct Protocol {
        float max_speed = 0, max_position = 0; // 原始速度、位置对称量程，rad/s、rad
    };

    struct Config {
        Motor::Config motor;
        Protocol protocol;
        float enable_frequency = 10; // 使能帧发送频率，Hz
    };

    enum ErrCode {
        NONE = 0x00,
        ENABLED = 0x01,
        OVP = 0x08,       // 超压
        UVP = 0x09,       // 欠压
        OCP = 0x0A,       // 过电流
        MOS_OTP = 0x0B,   // MOS 过温
        MOTOR_OTP = 0x0C, // 电机过温
        COMM_LOST = 0x0D, // 通讯丢失
        OVERLOAD = 0x0E,  // 过载
    } err_code = NONE;

    struct {
        float mos = 0, motor = 0; // ℃
    } temperature;

    void send();

  protected:
    explicit DmMotor(const Config& config);

  private:
    const Protocol protocol_;
    const float enable_frequency_;
    Timer enable_timer_;

    void can_callback(const CanFrame& frame);
    void send_mit() const;
    void send_enable() const;
    void send_disable() const;
};
