#pragma once

#include "motor/motor.hpp"

template <float PMAX, float VMAX, float TMAX>
class DmMotor : public Motor {
  public:
    enum ErrCode {
        NONE = 0x00,
        ENABLED = 0x01,
        ENC_UNCALIBRATED = 0x02,   // 输出轴编码器未校准
        ENC_CALIB_HIGH = 0x03,     // 输出轴编码器校准偏大
        ENC_DEVIATION_HIGH = 0x04, // 输出轴编码器校准偏差过大
        SENSOR_READ_ERR = 0x05,    // 读取传感器错误
        PARAM_READ_ERR = 0x07,     // 读取电机参数错误
        OVP = 0x08,                // 超压
        UVP = 0x09,                // 欠压
        OCP = 0x0A,                // 过电流
        MOS_OTP = 0x0B,            // MOS 过温
        MOTOR_OTP = 0x0C,          // 电机线圈过温
        COMM_LOST = 0x0D,          // 通讯丢失
        OVERLOAD = 0x0E,           // 过载
    } err_code = NONE;

    float temp_mos = 0;
    float temp_motor = 0;

    void send();

  protected:
    explicit DmMotor(const Config& config);

  private:
    static constexpr float ENABLE_FREQUENCY = 10; // 使能帧发送频率，Hz

    Timer enable_timer_;

    void can_callback(const CanFrame& frame);

    void send_mit() const;
    void send_enable() const;
    void send_disable() const;
    void send_clear_err() const;

    static float decode(uint16_t value, float maximum, unsigned bits);
    static uint16_t encode(float value, float maximum, unsigned bits);
};

#include "dm_motor.tpp"
