#pragma once

#include "Chassis.hpp"
#include "motor/Motor.hpp"

class Chassis_DualSteer : public Chassis {
public:
    // 底盘电机
    struct motor_t {
        Motor &w1, &w2; // 轮电机（左前，右后）
        Motor &s1, &s2; // 舵电机（左前，右后）
    } motor;

    // 舵轮角度
    struct {
        Angle<deg> measure, ref;
    } s1, s2;

    Chassis_DualSteer(const config_t& config, const motor_t& motor);

    // 底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    static constexpr UnitFloat<> MIN_F = 1 * N; // 最小转舵力

    bool is_invert_f1 = false, is_invert_f2 = false;

    // 速度和力学正解
    void forward() override;

    // 速度和力学逆解
    void backward() override;

    // 功率控制，必须放在【更新电机之后】
    void powerControlAfterMotorUpdate();
};