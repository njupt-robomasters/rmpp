#pragma once

#include "gimbal.hpp"
#include "math/angle.hpp"
#include "motor/motor.hpp"

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

    GimbalDualYaw(Attitude& attitude, const Config& config, const Motors& motor);

    void load_yaw_offset(KvStore& kv_store) override;

    void save_yaw_offset(KvStore& kv_store) override;

    void set_yaw_zero() override;

    // 在自定义UI上显示电机连接状态
    void update_ui(RobotUi& ui) override;

  private:
    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    // 角度正解：电机角度 -> 云台姿态
    void forward() override;

    // 角度正解：云台姿态 -> 电机角度
    void backward() override;
};
