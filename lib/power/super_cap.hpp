#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <cstdint>

// BirdFly G474 超级电容控制器（njupt-robomasters/G474SuperCap）：CAN 协议、底盘功率上限与电容在线辨识
// 控制器只在收到命令时应答，100 ms 未收到命令关闭 DCDC；装超电时由调用方按固定周期持续调用 send()
// 未装超电时不调用 send()，超电保持离线，按 OFF 行为只用估计功率
// DCDC 使能后把电池侧功率稳定在目标值，负载超过目标时电容自动放电
// 国际单位制：功率 W，能量 J，电压 V，电流 A，电荷 C；电容电流与功率充电为正、放电为负
class SuperCap {
  public:
    struct Config {
        // 通信与应答
        Can can;
        uint32_t tx_id = 0x061, rx_id = 0x051;
        float timeout = 0.1f; // 应答超时判断断联，s

        float current_fc = 10; // 应答电流（chassis、cap 共用）低通截止频率，Hz
        float error_fc = 5;    // 功率偏差（实测 − 估计）低通截止频率，Hz

        // 电池侧功率上限随缓冲能量线性变化：cutoff 为零，满 60 J 为 gain_max 倍裁判上限
        struct {
            float cutoff = 30;     // 上限为零的缓冲能量，J
            float gain_max = 1.1f; // 满缓冲能量时相对裁判上限的倍数
        } buffer;

        // BOOST 补偿：电容电压在截止与满额之间线性，满额补偿功率为 max_current·V_CAP
        struct {
            float voltage_cutoff = 10; // 补偿为零的电容电压，V
            float voltage_full = 12;   // 补偿满额的电容电压，V
            float max_current = 10;    // 最大补偿电流，A
        } boost;

        // 容量辨识 RLS：窗口电荷达 min_charge 时更新一步，max_window 内未达则重新开窗
        // 即平均电流须超过 min_charge / max_window，避免电流零偏触发
        struct {
            float min_charge = 1; // 每步最小充放电量，C
            float max_window = 1; // 窗口最长时间，s
            float lambda = 0.98f; // 遗忘因子，协方差迹小于初值时生效
            float p_init = 100;   // 初始协方差对角元
        } rls;
    };

    enum Mode {
        OFF,     // 关闭 DCDC，只用估计功率
        MONITOR, // 关闭 DCDC，用实测修正估计
        CHARGE,  // 使能 DCDC，负载按电池侧上限限制，余量给电容充电
        BOOST,   // 使能 DCDC，负载上限叠加电容放电补偿
    };

    bool is_connect = false;

    // 应答电压
    struct {
        float chassis = 0; // 底盘口（电池侧）电压
        float cap = 0;     // 电容电压
    } voltage;

    // 应答电流，已低通
    struct {
        float chassis = 0; // 底盘负载电流
        float cap = 0;     // 电容电流
    } current;

    // 功率：实测满足 battery = chassis + cap（忽略 DCDC 损耗）
    // battery_limit 发给超电作电池侧目标；chassis_limit 为估计模型下的底盘功率上限，供底盘限流
    struct {
        float chassis = 0;       // 实测底盘功率
        float cap = 0;           // 实测电容功率
        float battery = 0;       // 实测电池侧功率，即裁判系统测量对象
        float estimate = 0;      // 底盘电机估计功率
        float error = 0;         // chassis − estimate，已低通；离线或 OFF 时为零
        float battery_limit = 0; // 电池侧功率上限，裁判上限按缓冲能量折算
        float boost_max = 0;     // 电容可补偿功率；非 BOOST 或离线时为零
        float chassis_limit = 0; // 底盘功率上限 = battery_limit + boost_max − error；不限功率时为无穷大
    } power;

    // 电容在线辨识：模型 V_CAP = Q/C + ESR·I_CAP；未收敛或无效时为零
    float capacitance = 0; // 电容量，F
    float resistance = 0;  // 等效串联电阻，Ω

    explicit SuperCap(const Config& config);

    // 设置模式
    void set_mode(Mode mode);

    // 设置裁判电池侧功率上限，65535 表示不限功率
    void set_battery_limit(float battery_limit);

    // 设置裁判缓冲能量，上限 60 J
    void set_buffer_energy(float buffer_energy);

    // 设置底盘电机估计功率
    void set_estimate(float estimate);

    // 每个控制周期调用：断联检测、电池侧功率上限、偏差修正与底盘功率上限；底盘失能时也调用
    void update();

    // 发送一帧命令；调用周期由调用方控制，须小于控制器 100 ms 心跳超时
    void send() const;

  private:
    static constexpr float BUFFER_FULL = 60;  // 裁判缓冲能量上限，J
    static constexpr float UNLIMITED = 65535; // 裁判不限功率时的上限读数

    const Config config_;
    Mode mode_ = OFF;
    float battery_limit_ = 120; // 裁判电池侧功率上限原始读数，W
    float buffer_energy_ = 60;  // 裁判缓冲能量，J
    Timer update_timer_;        // update() 间隔，用于偏差低通
    Timer callback_timer_;      // can_callback() 间隔，用于断联检测、电流低通与容量辨识

    // 容量辨识状态：窗口起点与窗口内累计量，参数 θ = [1/C, ESR] 与协方差 P
    struct {
        float last_current = 0; // 上帧原始电容电流，A
        float voltage = 0;      // 窗口起点电容电压，V
        float current = 0;      // 窗口起点原始电容电流，A
        float charge = 0;       // 窗口内累计电荷，C
        float time = 0;         // 窗口时长，s
        float theta[2] = {};
        float p[2][2] = {};
    } rls_;

    void can_callback(const CanFrame& frame);

    // 容量辨识一步；在 can_callback 中置 is_connect 前调用，current_cap 为未低通的电容电流
    void update_capacitance(float current_cap, float dt);
};
