#pragma once

#include "app/shooter.hpp"
#include "motor/m2006.hpp"
#include "motor/n630.hpp"

class Shooter : public Shooter_Template {
public:
    static constexpr UnitFloat WHEEL_RADIUS = 3.0f * cm;
    static constexpr float SHOOT_PRE_ROUND = 7.0f;

    // 电机对象
    M2006 m_shoot;
    N630 n630_1, n630_2;

    Shooter(PID::param_t* shoot_pid);

    void SetEnable(bool is_enable) override;

    void OnLoop() override;

private:
    // 速度正解：弹速、弹频 -> 电机速度
    void speedForward() override;

    // 速度逆解：电机速度 -> 弹速、弹频
    void speedBackward() override;
};