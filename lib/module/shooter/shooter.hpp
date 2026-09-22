#pragma once

#include "math/unit.hpp"
#include "ui/ui.hpp"

class Shooter {
  public:
    // 发射机构配置
    struct Config {
        float bullet_per_angle = 1.0f / rev; // shots/rad; default one shot per revolution
        float block_time = 500 * ms;         // 卡弹检测时间
        float reverse_time = 250 * ms;       // 卡弹倒转时间
        float rub_radius = 3.0f * cm;        // 摩擦轮半径
    } config;

    bool is_enable = false;       // 使能标志
    bool is_rub = false;          // 摩擦轮状态
    bool is_shoot = false;        // 拨弹电机状态
    bool is_heat_protect = false; // 热量保护状态

    // 弹速 (m/s)
    struct {
        float ref{}, measure{};
    } bullet_speed;

    // 弹频 (shots/s)
    struct {
        float ref{}, measure{};
    } bullet_freq;

    // 拨弹电机电流
    float shoot_current{};

    Shooter(const Config& config);

    // 设置发射结构使能/失能
    virtual void set_enable(bool is_enable);

    // 设置弹速
    void set_bullet_speed(float bullet_speed);

    // 设置弹频
    void set_bullet_freq(float bullet_freq);

    // 设置摩擦轮
    void set_rub(bool is_rub);

    // 设置拨弹电机
    void set_shoot(bool is_shoot);

    // 设置热量保护
    void set_heat_protect(bool is_heat_protect);

    // 在自定义UI上显示电机连接状态
    virtual void update_ui(Ui& ui) = 0;

    // 需要在循环中调用
    virtual void on_loop();

  protected:
    bool is_block_ = false; // 堵转标志
    Timer timer_block_;     // 堵转计时器

    // 正解：电机速度 -> 弹速、弹频
    virtual void forward() = 0;

    // 逆解：弹速、弹频 -> 电机速度
    virtual void backward() = 0;
};
