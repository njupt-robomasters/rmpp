#pragma once

#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include "motor/vesc.hpp"
#include "pid/pid.hpp"

// 17mm 发射机构：双 VESC 摩擦轮速度模式，拨弹电机速度闭环；摩擦轮与拨弹电机各自处理掉线
class Shooter17mm {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s；避免首次调用和停顿后的大步长

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

    // 弹速 (m/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_speed;

  private:
    // 弹频 (shots/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_freq;

  public:
    // 摩擦轮线速度，m/s
    float rub_speed[2]{};

  private:
    // 拨弹闭环与弹数换算
    const PidOutput shoot_output;
    const float bullet_per_angle; // 每弧度发射弹数，默认每转一发

    // 卡弹与摩擦轮参数
    const float block_time;      // 卡弹检测时间
    const float reverse_time;    // 卡弹倒转时间
    const float rub_radius;      // 摩擦轮半径
    const uint16_t heat_protect; // 剩余热量保护阈值

    // 设备绑定
    struct Motors {
        Vesc &rub1, &rub2; // 摩擦轮
        Motor& shoot;      // 拨弹电机
    } motor;

    // 指令
    bool is_enable = false;
    bool is_heat_protect = false; // 热量保护状态
    bool is_block = false;        // 卡弹标志

    // 拨弹闭环与卡弹检测
    Pid shoot_pid;
    Timer block_timer; // 卡弹计时器
    Timer update_timer;

  public:
    Shooter17mm(const Motors& motor, const Config& config);

    // 设置发射机构使能/失能
    void set_enable(bool is_enable);

    // 设置弹速参考值，m/s；0 关闭摩擦轮，保持到下次设置
    void set_bullet_speed(float bullet_speed);

    // 设置弹频参考值，发/秒；0 停止拨弹，负值反转，保持到下次设置
    void set_bullet_freq(float bullet_freq);

    // 设置裁判剩余热量，允许过热时的负值；未设置时不限制
    void set_heat_remain(int32_t heat_remain);

    // 每个控制周期调用
    void update();
};
