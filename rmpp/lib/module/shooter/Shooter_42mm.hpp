#pragma once

#include "Shooter.hpp"
#include "motor/Motor.hpp"

class Shooter_42mm : public Shooter {
public:
    // 发射机构电机
    struct motor_t {
        Motor &rub_left1, &rub_right1, &rub_left2, &rub_right2; // 摩擦轮
        Motor& shoot;                                           // 拨弹电机
    } motor;

    UnitFloat<m_s> rub_left1_measure, rub_right1_measure, rub_left2_measure, rub_right2_measure;

    Shooter_42mm(const config_t& config, const motor_t& motor);

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