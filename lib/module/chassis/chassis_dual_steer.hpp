#pragma once

#include "chassis.hpp"
#include "math/angle.hpp"
#include "math/conversions.hpp"
#include "motor/motor.hpp"

using math::Angle;
using math::cm;

class ChassisDualSteer : public Chassis {
  public:
    // 底盘电机
    struct Motors {
        Motor &w1, &w2; // 轮电机（左前，右后）
        Motor &s1, &s2; // 舵电机（左前，右后）
    } motor;

    // 舵轮角度
    struct {
        Angle measure, ref_v, ref_f;
    } s1, s2;

    ChassisDualSteer(const Config& config, const Motors& motor);

    // 底盘使能/失能
    void set_enable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void update_ui(Ui& ui) override;

    // 需要在循环中调用
    void on_loop() override;

  private:
    static constexpr float MIN_V = 1 * (cm); // 最小转舵速度
    static constexpr float MIN_F = 1;        // 最小转舵力

    bool is_invert_v1_ = false, is_invert_v2_ = false;
    bool is_invert_f1_ = false, is_invert_f2_ = false;

    // 速度和力学正解
    void forward() override;

    // 速度和力学逆解
    void backward() override;

    // 功率控制，必须放在【更新电机之后】
    void power_control_after_motor_update();
};
