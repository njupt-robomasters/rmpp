#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <cstdint>

// BirdFly G474 超级电容控制器（njupt-robomasters/G474SuperCap）：CAN 协议、底盘功率管理与电容在线辨识
// 控制器只在收到命令时应答，100 ms 未收到命令关闭 DCDC；装超电时由调用方按固定周期持续调用 send()
// 未装超电时不调用 send()，超电保持离线，按 OFF 用裁判缓冲能量
// DCDC 使能后把电池侧功率稳定在目标值，负载超过目标时电容自动放电
// 功率管理：裁判上限按裁判缓冲能量折算为电池侧目标下发超电；底盘功率目标为供给功率按所用缓冲能量折算
// DCDC 使能时用虚拟缓冲能量，按（供给 − 底盘实测）积分；否则用裁判缓冲能量
// 国际单位制：功率 W，能量 J，电压 V，电流 A，电荷 C；电容电流与功率充电为正、放电为负
class SuperCap {
  private:
    static constexpr float BUFFER_FULL = 60;  // 裁判缓冲能量上限，J
    static constexpr float UNLIMITED = 65535; // 裁判不限功率时的上限读数
    static constexpr float DT_MAX = 0.1f;     // update() 积分步长上限，s；避免首次调用的大间隔

  public:
    enum Mode {
        OFF,    // 关闭 DCDC，用裁判缓冲能量；应答解算与容量辨识照常
        CHARGE, // 使能 DCDC，底盘供给为电池侧目标，余量给电容充电
        BOOST,  // 使能 DCDC，底盘供给叠加电容放电补偿
    };

    struct Config {
        // 通信与应答
        Can can;
        uint32_t tx_id = 0x061, rx_id = 0x051;
        float timeout = 0.1f;  // 应答超时判断断联，s
        float current_fc = 10; // 应答电流（chassis、cap 共用）低通截止频率，Hz

        // 缓冲能量折算倍数：cutoff 为零，满 60 J 为 gain_max，其间线性；裁判与虚拟缓冲能量共用

        float buffer_cutoff = 30;     // 倍数为零的缓冲能量，J
        float buffer_gain_max = 1.1f; // 满缓冲能量时的倍数

        // BOOST 补偿：电容电压在截止与满额之间线性，满额补偿功率为 max_current·V_CAP

        float boost_voltage_cutoff = 10; // 补偿为零的电容电压，V
        float boost_voltage_full = 12;   // 补偿满额的电容电压，V
        float boost_max_current = 10;    // 最大补偿电流，A

        // 容量辨识 RLS：窗口电荷达 min_charge 时更新一步，max_window 内未达则重新开窗
        // 即平均电流须超过 min_charge / max_window，避免电流零偏触发
        struct {
            float min_charge = 1; // 每步最小充放电量，C
            float max_window = 1; // 窗口最长时间，s
            float lambda = 0.98f; // 遗忘因子，协方差迹小于初值时生效
            float p_init = 100;   // 初始协方差对角元
        } rls = {};
    };

  private:
    bool is_connect = false;

    // 应答电压
    float voltage_chassis = 0; // 底盘口（电池侧）电压
    float voltage_cap = 0;     // 电容电压

    // 应答电流，已低通
    float current_chassis = 0; // 底盘负载电流
    float current_cap = 0;     // 电容电流

  public:
    // 功率：实测满足 battery = chassis + cap（忽略 DCDC 损耗）；不限功率时两个目标为无穷大
    float power_chassis = 0;     // 实测底盘功率
    float power_cap = 0;         // 实测电容功率
    float power_battery = 0;     // 实测电池侧功率，即裁判系统测量对象
    float power_battery_ref = 0; // 电池侧目标，下发超电；裁判上限按裁判缓冲能量折算

  private:
    float power_boost_max = 0; // 电容可补偿功率；非 BOOST 或离线时为零

  public:
    float power_chassis_ref = 0; // 底盘功率目标，供底盘限流；供给功率按所用缓冲能量折算

  private:
    // 缓冲能量，J
    float energy_referee = 60; // 裁判缓冲能量

  public:
    float energy_local = 60; // 虚拟缓冲能量；DCDC 使能时积分，否则等于 referee

    // 电容在线辨识：模型 V_CAP = Q/C + ESR·I_CAP；未收敛或无效时为零
    float capacitance = 0; // 电容量，F
    float resistance = 0;  // 等效串联电阻，Ω

  private:
    const Can can;
    const uint32_t tx_id, rx_id;
    const float timeout, current_fc;

    // 缓冲折算与 BOOST 补偿
    const float buffer_cutoff, buffer_gain_max;
    const float boost_voltage_cutoff, boost_voltage_full, boost_max_current;

    // 指令
    Mode mode = OFF;
    float referee_limit = 120; // 裁判电池侧功率上限原始读数，W

    // 容量辨识状态：窗口起点与窗口内累计量，参数 θ = [1/C, ESR] 与协方差 P
    struct {
        const float min_charge = 1; // 每步最小充放电量，C
        const float max_window = 1; // 窗口最长时间，s
        const float lambda = 0.98f; // 遗忘因子，协方差迹小于初值时生效
        const float p_init = 100;   // 初始协方差对角元

        float last_current = 0; // 上帧原始电容电流，A
        float voltage = 0;      // 窗口起点电容电压，V
        float current = 0;      // 窗口起点原始电容电流，A
        float charge = 0;       // 窗口内累计电荷，C
        float time = 0;         // 窗口时长，s
        float theta[2] = {};
        float p[2][2] = {};
    } rls;

    Timer update_timer;   // update() 间隔，用于虚拟缓冲能量积分
    Timer callback_timer; // can_callback() 间隔，用于断联检测、电流低通与容量辨识

  public:
    explicit SuperCap(const Config& config);

    // 设置模式
    void set_mode(Mode mode);

    // 设置裁判电池侧功率上限，65535 表示不限功率
    void set_referee_limit(float referee_limit);

    // 设置裁判缓冲能量，上限 60 J
    void set_referee_energy(float referee_energy);

    // 每个控制周期调用：断联检测、电池侧目标、虚拟缓冲能量与底盘功率目标；底盘失能时也调用
    void update();

    // 发送一帧命令；调用周期由调用方控制，须小于控制器 100 ms 心跳超时
    void send() const;

  private:
    void can_callback(const CanFrame& frame);

    // DCDC 是否使能：在线且 CHARGE 或 BOOST
    bool is_dcdc_enable() const;

    // 缓冲能量折算倍数
    float buffer_gain(float energy) const;

    // 容量辨识一步；在 can_callback 中置 is_connect 前调用，current_cap 为未低通的电容电流
    void update_capacitance(float current_cap, float dt);
};
