#pragma once

#include "pid.hpp"
#include "m2006.hpp"
#include "n630.hpp"

extern "C" void task_can_entry(const void* argument);

class Shooter {
    friend void task_can_entry(const void* argument);

private:
    static constexpr UnitFloat<mm> WHEEL_RADIUS = 30.0f * mm;
    static constexpr float SHOOT_PRE_ROUND = 7.0f;

    // 电机对象
    M2006 m_shoot;
    N630 n630_1, n630_2;

    // 使能标志
    bool is_enable = false;

    // 拨弹电机状态
    bool is_prepare_shoot = false;
    struct {
        UnitFloat<Hz> ref, measure;
    } shoot_freq;

    // 摩擦轮状态
    bool is_shoot = false;
    struct {
        UnitFloat<m_s> ref, measure1, measure2;
    } bullet_speed;

    void forwardCalc();

    void backwardCalc();

public:
    Shooter(PID::param_t* shoot_pid_param);

    void SetEnable(bool is_enable);

    void SetBulletSpeed(const UnitFloat<>& bullet_speed);

    void SetShootFreq(const UnitFloat<>& shoot_freq);

    void SetPrepareShoot(bool is_prepare_shoot);

    void SetShoot(bool is_shoot);

    void Update();
};
