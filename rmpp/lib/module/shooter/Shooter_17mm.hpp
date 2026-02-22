#pragma once

#include "Shooter.hpp"
#include "motor/Motor.hpp"

class Shooter_17mm : public Shooter {
public:
    // 发射机构电机
    struct motor_t {
        Motor &rub1, &rub2; // 摩擦轮
        Motor& shoot;       // 拨弹电机
    } motor;

    // 摩擦轮线速度
    UnitFloat<m_s> rub1_measure, rub2_measure;

    Shooter_17mm(const config_t& config, const motor_t& motor);

    // 设置发射结构使能/失能
    void SetEnable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void HandleUI(UI& ui) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 正解：电机速度 -> 弹速、弹频
    void forward() override;

    // 逆解：弹速、弹频 -> 电机速度
    void backward() override;
};