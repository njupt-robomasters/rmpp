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

    // 轮子线速度
    struct {
        UnitFloat<m_s> ref, measure;
    } v1, v2;

    // 轮子力
    struct {
        UnitFloat<m_s> ref, measure;
    } f1, f2;

    Chassis_DualSteer(const config_t& config, const motor_t& motor);

    // 底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    static constexpr UnitFloat<> MIN_V = 0.01f * m_s; // 最小转舵速度

    void backward() override;

    void forward() override;
};