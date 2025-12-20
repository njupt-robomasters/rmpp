#pragma once

#include "controller/pid.hpp"

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
    } bullet_freq;

    UnitFloat<A> shoot_current;

    // 设置发射结构使能/失能
    virtual void SetEnable(bool is_enable);

    // 设置弹速
    void SetBulletSpeed(const UnitFloat<>& bullet_speed);

    // 设置射频
    void SetBulletFreq(const UnitFloat<>& bullet_freq);

    // 设置摩擦轮
    void SetPrepareShoot(bool is_prepare_shoot);

    // 设置拨弹电机
    void SetShoot(bool is_shoot);

    // 需要在循环中调用
    virtual void OnLoop();

private:
    // 速度正解：弹速、弹频 -> 电机速度
    virtual void speedForward() = 0;

    // 速度逆解：电机速度 -> 弹速、弹频
    virtual void speedBackward() = 0;
};
