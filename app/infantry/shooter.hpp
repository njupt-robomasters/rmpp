#pragma once

#include "app/shooter.hpp"
#include "motor/m2006.hpp"
#include "motor/n630.hpp"

class Shooter : public Shooter_Template {
public:
    static constexpr UnitFloat WHEEL_RADIUS = 3.0f * cm;
    static constexpr float SHOOT_PRE_ROUND = 9.0f / 2.5f;

    // 电机对象
    M2006 m_shoot;
    N630 n630_1, n630_2;

    Shooter(PID::param_t* shoot_pid);

    void SetEnable(bool is_enable) override;

    void OnLoop() override;

private:
    void forwardCalc() override;

    void backwardCalc() override;
};