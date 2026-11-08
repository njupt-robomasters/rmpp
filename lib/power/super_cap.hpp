#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <cstdint>

// BirdFly G474 超级电容控制器（njupt-robomasters/G474SuperCap）与底盘功率上限
// 控制器只在收到命令时应答，100 ms 未收到命令关闭 DCDC；装超电时由调用方按固定周期持续调用 send()
// 未装超电时不调用 send()，超电保持离线，按 OFF 行为只用估计功率
// DCDC 使能后把电池侧功率稳定在目标值，负载超过目标时电容自动放电
// 国际单位制：功率 W，能量 J，电压 V，电流 A
class SuperCap {
  public:
    struct Config {
        Can can;
        uint32_t tx_id = 0x061, rx_id = 0x051;
        float timeout = 0.1f;             // 应答超时，s
        float current_fc = 10;            // 应答电流（out、cap 共用）低通截止频率，Hz
        float buffer_gain_max = 1.1f;     // 缓冲能量满 60 J 时电池侧功率上限相对裁判上限的倍数；30 J 为零，其间线性
        float error_fc = 5;               // 功率偏差（实测 − 估计）低通截止频率，Hz
        float voltage_cutoff = 10;        // BOOST 补偿为零的电容电压，V
        float voltage_full = 12;          // BOOST 补偿满额的电容电压，V；与截止电压之间线性
        float max_discharge_current = 10; // 最大补偿电流，A，BOOST 满额补偿功率为此值·电容电压
    };

    enum Mode {
        OFF,     // 关闭 DCDC，不用实测功率
        MONITOR, // 关闭 DCDC，用实测修正估计
        CHARGE,  // 使能 DCDC，功率不超过电池侧上限，只充电
        BOOST,   // 使能 DCDC，允许电容放电补偿
    };

    bool is_connect = false;

    // 应答测量
    struct {
        float out = 0; // 底盘口（电池侧）电压
        float cap = 0; // 电容电压
    } voltage;

    // 应答电流，已低通
    struct {
        float out = 0; // 底盘负载电流
        float cap = 0; // 电容电流，充电为正
    } current;

    // 功率：实测满足 battery = out + cap（忽略 DCDC 损耗）
    // battery_limit 发给超电作电池侧目标；ref 为本机估计模型下的功率控制目标 = battery_limit + boost − error
    struct {
        float out = 0;           // 实测底盘负载功率
        float cap = 0;           // 实测电容功率，充电为正、放电为负
        float battery = 0;       // 电池侧功率 = out + cap，即裁判系统测量对象
        float estimate = 0;      // 底盘电机估计功率
        float error = 0;         // 低通后的实测 − 估计；离线或 OFF 时为零
        float boost = 0;         // BOOST 时电容可补偿功率，否则为零
        float battery_limit = 0; // 裁判上限按缓冲能量折算的电池侧功率上限
        float ref = 0;           // 功率控制目标；不限功率时为无穷大
    } power;

    explicit SuperCap(const Config& config);

    // 设置模式
    void set_mode(Mode mode);

    // 设置裁判电池侧功率上限，65535 表示不限功率
    void set_battery_limit(float battery_limit);

    // 设置裁判缓冲能量，上限 60 J
    void set_buffer_energy(float buffer_energy);

    // 设置底盘电机估计功率
    void set_estimate(float estimate);

    // 每个控制周期调用：应答超时、电池侧功率上限、偏差修正与功率控制目标；底盘失能时也调用
    void update();

    // 发送一帧命令；调用周期由调用方控制，须小于控制器 100 ms 心跳超时
    void send() const;

  private:
    static constexpr float BUFFER_CUTOFF = 30; // 缓冲能量低于此值电池侧上限为零，J
    static constexpr float BUFFER_FULL = 60;   // 裁判缓冲能量上限，J
    static constexpr float UNLIMITED = 65535;  // 裁判不限功率时的上限读数

    const Config config_;
    Mode mode_ = OFF;
    float battery_limit_ = 120;
    float buffer_energy_ = 60;
    Timer rx_timer_; // 应答超时
    Timer timer_;    // 偏差低通周期

    void can_callback(const CanFrame& frame);
};
