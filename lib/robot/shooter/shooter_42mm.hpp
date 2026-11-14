#pragma once

#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include "motor/vesc.hpp"
#include "pid/pid.hpp"
#include <cstdint>

// 42mm 发射机构：四 VESC 摩擦轮速度模式，拨弹电机位置闭环，发射信号上升沿拨一发，空闲时抖动下弹
class Shooter42mm {
  public:
    static constexpr float RUB12_RATIO = 0.5f; // 摩擦轮 1、2 相对弹速的速度比

  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // 控制周期限幅，s；避免首次调用和停顿后的大步长
    static constexpr float SHOOT_FINISH_RATIO = 0.95f;       // 用于检查拨弹完成
    static constexpr float SHAKE_FREQ = 1;                   // 抖动频率
    static constexpr float SHAKE_AMP = 1;                    // 抖动幅度（一个弹 = 1）

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
        float rub_radius = 3.0f * cm;        // 摩擦轮半径
        uint16_t heat_protect = 0;           // 剩余热量保护阈值
    };

    // 弹速 (m/s)
    struct {
        float ref = 0, measure = 0;
    } bullet_speed;

    // 摩擦轮线速度，m/s
    float rub_speed[4]{};

  private:
    const Config config;

    // 设备绑定
    struct Motors {
        Vesc &rub1, &rub2, &rub3, &rub4; // 摩擦轮
        Motor& shoot;                    // 拨弹电机
    } motor;

    // 指令；发射信号保留上周期值用于检测上升沿
    bool is_enable = false;
    bool is_heat_protect = false; // 热量保护状态
    bool is_shoot = false;

    // 拨弹状态机
    enum {
        FREE,
        SHOOT,
        SHAKE,
    } state = FREE;

    Angle shake_upper; // 抖动上沿角度
    bool last_shoot = false;
    Pid shoot_pid;
    Timer shake_timer;
    Timer block_timer; // 卡弹计时器
    Timer update_timer;

  public:
    Shooter42mm(const Motors& motor, const Config& config);

    // 设置发射机构使能/失能
    void set_enable(bool is_enable);

    // 设置弹速参考值，m/s；0 关闭摩擦轮，保持到下次设置
    void set_bullet_speed(float bullet_speed);

    // 设置发射信号，上升沿拨一发
    void set_shoot(bool is_shoot);

    // 设置裁判剩余热量，允许过热时的负值；未设置时不限制
    void set_heat_remain(int32_t heat_remain);

    // 每个控制周期调用
    void update();

  private:
    // 按配置输出类型设置拨弹电机电流或力矩，并用限幅后实际输出跟踪 PID
    void set_shoot_output(float output);
};
