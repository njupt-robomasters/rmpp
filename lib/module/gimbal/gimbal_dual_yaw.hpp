#pragma once

#include "gimbal.hpp"
#include "math/angle.hpp"
#include "motor/motor.hpp"

using math::Angle;

class GimbalDualYaw : public Gimbal {
  public:
    // 云台电机
    struct Motors {
        Motor &yaw1, &yaw2, &pitch; // yaw1-大yaw，yaw2-小yaw
    } motor;

    // 大小yaw分配
    struct {
        Angle ref, measure;
    } yaw1, yaw2; // yaw1-大yaw，yaw2-小yaw

    GimbalDualYaw(Imu& imu, const Motors& motor);

    // 云台使能/失能
    void set_enable(bool is_enable) override;

    void load_yaw_offset(FlashDb& flashdb) override;

    void save_yaw_offset(FlashDb& flashdb) override;

    void set_yaw_zero() override;

    // 在自定义UI上显示电机连接状态
    void update_ui(Ui& ui) override;

    // 需要在循环中调用
    void on_loop() override;

  private:
    // 角度正解：电机角度 -> 云台姿态
    void forward() override;

    // 角度正解：云台姿态 -> 电机角度
    void backward() override;
};
