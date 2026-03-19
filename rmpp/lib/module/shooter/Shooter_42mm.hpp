#pragma once

#include "Shooter.hpp"
#include "motor/Motor.hpp"

class Shooter_42mm : public Shooter {
public:
    // 发射机构电机
    struct motor_t {
        Motor &rub1, &rub2, &rub3, &rub4; // 摩擦轮
        Motor& shoot;                     // 拨弹电机
    } motor;

    Shooter_42mm(const config_t& config, const motor_t& motor);

    // 设置发射结构使能/失能
    void SetEnable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void UpdateUI(UI& ui) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    static constexpr UnitFloat<> RUB12_RATIO = 0.5f * ratio;
    static constexpr UnitFloat<> SHOOT_FINISH_ERR = 1 * deg; // 用于检查拨弹完成
    static constexpr UnitFloat<> SHAKE_FREQ = 1 * Hz;        // 抖动频率
    static constexpr float SHANE_AMP = 1;                    // 抖动幅度（一个弹 = 1）

    // 摩擦轮线速度
    UnitFloat<m_s> rub1_measure, rub2_measure, rub3_measure, rub4_measure;

    // 用于检测发送信号上升沿
    bool is_shoot_last = false;

    // 用于抖动下弹
    Angle<deg> shake_upper;
    BSP::Dwt dwt_shake;

    enum {
        FREE,
        SHOOT,
        SHAKE,
    } state = FREE;

    // 正解：电机速度 -> 弹速、弹频
    void forward() override;

    // 逆解：弹速、弹频 -> 电机速度
    void backward() override;
};