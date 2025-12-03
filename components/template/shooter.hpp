#pragma once

#include "pid.hpp"

class Shooter_Template {
public:
    // 使能标志
    bool is_enable = false;

    // 摩擦轮状态
    bool is_prepare_shoot = false;

    // 拨弹电机状态
    bool is_shoot = false;

    // 弹速
    struct {
        UnitFloat<m_s> ref, measure1, measure2;
    } bullet_speed;

    // 射频
    struct {
        UnitFloat<Hz> ref, measure;
    } shoot_freq;

    // 设置弹速
    void SetBulletSpeed(const UnitFloat<>& bullet_speed);

    // 设置单频
    void SetShootFreq(const UnitFloat<>& shoot_freq);

    // 设置摩擦轮
    void SetPrepareShoot(bool is_prepare_shoot);

    // 设置拨弹电机
    void SetShoot(bool is_shoot);

    // 设置发射结构使能/失能
    virtual void SetEnable(bool is_enable) = 0;

    // 需要在循环中调用
    virtual void OnLoop() = 0;

private:
    virtual void forwardCalc() = 0;

    virtual void backwardCalc() = 0;
};
