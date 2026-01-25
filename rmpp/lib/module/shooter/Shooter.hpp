#pragma once

#include "algorithm/PID.hpp"

class Shooter {
public:
    // 发射机构配置
    struct config_t {
        UnitFloat<> bullet_per_rev = 1.0f * (Hz / rev); // 拨弹盘转一圈的发弹数
        UnitFloat<> rub_radius = 3.0f * cm;             // 摩擦轮半径
    } config;

    bool is_enable = false; // 使能标志
    bool is_rub = false;    // 摩擦轮状态
    bool is_shoot = false;  // 拨弹电机状态

    // 弹速
    struct {
        struct {
            UnitFloat<m_s> left, right;
        } measure;

        UnitFloat<m_s> ref;
    } bullet_speed;

    // 弹频
    struct {
        UnitFloat<Hz> ref, measure;
    } bullet_freq;

    Shooter(const config_t& config);

    // 设置发射结构使能/失能
    virtual void SetEnable(bool is_enable);

    // 设置弹速
    void SetBulletSpeed(const UnitFloat<>& bullet_speed);

    // 设置弹频
    void SetBulletFreq(const UnitFloat<>& bullet_freq);

    // 设置摩擦轮
    void SetRub(bool is_rub);

    // 设置拨弹电机
    void SetShoot(bool is_shoot);

    // 读取拨弹电机电流，显示在自定义UI上判断堵转
    virtual UnitFloat<> GetShootCurrentMeasure() = 0;

    // 需要在循环中调用
    virtual void OnLoop();

protected:
    // 速度正解：弹速、弹频 -> 电机速度
    virtual void speedForward() = 0;

    // 速度逆解：电机速度 -> 弹速、弹频
    virtual void speedBackward() = 0;
};