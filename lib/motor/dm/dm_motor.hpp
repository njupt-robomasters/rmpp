#pragma once

#include "motor/mit_motor.hpp"

class DmMotor : public MitMotor {
  public:
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

    void set_enable(bool enable) override;
    bool send_can_cmd() override;

  protected:
    DmMotor(const Config& config, Protocol protocol);

  private:
    unsigned send_count_ = 0;
    bool enable_submitted_ = false;

    void receive(const CanFrame& frame) final;
};
