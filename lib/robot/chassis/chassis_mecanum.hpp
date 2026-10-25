#pragma once

#include "chassis.hpp"
#include "motor/motor.hpp"

class ChassisMecanum : public Chassis {
  public:
    // 底盘电机
    struct Motors {
        Motor &w1, &w2, &w3, &w4; // 电机（左前，左后，右后，右前）
    } motor;

    ChassisMecanum(const Config& config, const Motors& motor);

    // 设置底盘使能/失能
    void set_enable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void update_ui(RobotUi& ui) override;

    // 需要在循环中调用
    void update() override;

  private:
    // 速度和力学正解
    void forward() override;

    // 速度和力学逆解
    void backward() override;
};
