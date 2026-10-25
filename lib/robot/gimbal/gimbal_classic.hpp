#pragma once

#include "gimbal.hpp"
#include "motor/motor.hpp"

class GimbalClassic : public Gimbal {
  public:
    // 云台电机
    struct Motors {
        Motor &yaw, &pitch;
    } motor;

    GimbalClassic(Imu& imu, const Config& config, const Motors& motor);

    // 云台使能/失能
    void set_enable(bool is_enable) override;

    void load_yaw_offset(Storage& storage) override;

    void save_yaw_offset(Storage& storage) override;

    void set_yaw_zero() override;

    // 在自定义UI上显示电机连接状态
    void update_ui(RobotUi& ui) override;

    // 需要在循环中调用
    void update() override;

  private:
    // 角度正解：电机角度 -> 云台姿态
    void forward() override;

    // 角度正解：云台姿态 -> 电机角度
    void backward() override;
};
