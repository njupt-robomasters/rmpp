#pragma once

#include "pid.hpp"
#include "m2006.hpp"
#include "n630.hpp"

extern "C" void task_can_entry(const void* argument);

class Shooter {
    friend void task_can_entry(const void* argument);

private:
    static constexpr Unit<mm> WHEEL_RADIUS = 30.0f * mm;
    static constexpr float SHOOT_PRE_ROUND = 7.0f;

    // 电机对象
    M2006 m2006;
    N630 n630_1, n630_2;

    // 使能标志
    bool is_enable = false;
    bool is_prepare_shoot = false;
    bool is_shoot = false;

    // 摩擦轮状态
    Unit<m_s> bullet_speed;
    Unit<rpm> n630_speed;

    // 拨弹电机状态
    struct {
        Unit<Hz> ref, measure;
    } shoot_freq;

public:
    Shooter(PID::param_t* m2006_pid);

    void SetEnable(bool is_enable);

    void SetBulletSpeed(Unit<m_s> bullet_speed);

    void SetShootFreq(Unit<Hz> shoot_freq);

    void SetPrepareShoot(bool is_prepare_shoot);

    void SetShoot(bool is_shoot);

    void Update();
};
