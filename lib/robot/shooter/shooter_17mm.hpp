#pragma once

#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "motor/vesc.hpp"
#include "pid/pid.hpp"
#include <cstdint>

// 17mm 发射机构：双 VESC 摩擦轮速度模式，拨弹电机速度闭环；摩擦轮与拨弹电机各自处理掉线
class Shooter17mm {
  public:
    enum PidOutput {
        CURRENT_OUTPUT,
        TORQUE_OUTPUT
    };

    // 发射机构配置
    struct Config {
        const Pid::Config* shoot_pid_config;
        PidOutput shoot_output = CURRENT_OUTPUT;
        float bullet_per_angle = 1.0f / rev; // 每弧度发射弹数，默认每转一发
        float block_time = 500 * ms;         // 卡弹检测时间
        float reverse_time = 250 * ms;       // 卡弹倒转时间
        float rub_radius = 3.0f * cm;        // 摩擦轮半径
        uint16_t heat_protect = 0;           // 剩余热量保护阈值
    };

    // 发射机构电机
    struct Motors {
        Vesc &rub1, &rub2; // 摩擦轮
        Motor& shoot;      // 拨弹电机
    } motor;

    // 弹速 (m/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_speed;

    // 弹频 (shots/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_freq;

    // 摩擦轮线速度，m/s
    float rub_speed[2]{};

    Shooter17mm(const Motors& motor, const Config& config);

    // 设置发射机构使能/失能
    void set_enable(bool is_enable);

    // 设置弹速参考值，m/s；0 关闭摩擦轮，保持到下次设置
    void set_bullet_speed(float bullet_speed);

    // 设置弹频参考值，发/秒；0 停止拨弹，负值反转，保持到下次设置
    void set_bullet_freq(float bullet_freq);

    // 传入裁判剩余热量，允许过热时的负值；未传入时不限制
    void set_heat_remain(int32_t heat_remain);

    // 需要在循环中调用
    void update();

  private:
    const Config config_;
    bool is_enable_ = false;
    bool is_heat_protect_ = false; // 热量保护状态
    Pid shoot_pid_;
    bool is_block_ = false; // 卡弹标志
    Timer timer_block_;     // 卡弹计时器
};
