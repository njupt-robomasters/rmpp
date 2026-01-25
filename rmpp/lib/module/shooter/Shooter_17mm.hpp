#pragma once

#include "Shooter.hpp"
#include "motor/Motor.hpp"

class Shooter_17mm : public Shooter {
public:
    // 发射机构电机
    struct motor_t {
        Motor &rub_left, &rub_right; // 摩擦轮
        Motor& shoot;                // 拨弹电机
    } motor;

    Shooter_17mm(const config_t& config, const motor_t& motor);

    // 设置发射结构使能/失能
    void SetEnable(bool is_enable) override;

    // 读取拨弹电机电流，显示在自定义UI上判断堵转
    UnitFloat<> GetShootCurrentMeasure() override { return motor.shoot.current.measure; }

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 速度正解：弹速、弹频 -> 电机速度
    void speedForward() override;

    // 速度逆解：电机速度 -> 弹速、弹频
    void speedBackward() override;
};