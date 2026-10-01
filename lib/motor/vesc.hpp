#pragma once

#include "math/unit.hpp"
#include "motor.hpp"

// 只支持开环速度控制：set_speed 后由 send 下发 eRPM，update 仍需每周期调用以刷新在线状态
class Vesc : public Motor {
  public:
    // 公共电机配置与 VESC 专用参数
    struct Config {
        Motor::Config motor;
        uint8_t pole_pair = 7;                // 电机极对数
        float auto_release_speed = 120 * rpm; // 零目标时低于该转速自动释放，rad/s；零禁用
    };

    struct {
        float mos = 0, motor = 0; // ℃
    } temperature;

    struct {
        float voltage = 0, current = 0; // 母线电压、电流，V、A
    } input;

    float duty = 0; // 占空比

    explicit Vesc(const Config& config);

    void send() const;

  private:
    // VESC 专用配置
    const uint8_t pole_pair_;
    const float auto_release_speed_;

    enum Command {
        CAN_PACKET_SET_CURRENT_BRAKE = 2,
        CAN_PACKET_SET_RPM = 3
    };

    enum Status {
        CAN_PACKET_STATUS = 9,
        CAN_PACKET_STATUS_4 = 16,
        CAN_PACKET_STATUS_5 = 27
    };

    void can_callback(const CanFrame& frame);
    // 发送 4 字节大端定点数命令
    void send_command(Command command, float value, float scale) const;
};
