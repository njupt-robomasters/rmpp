#pragma once

#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include <cstdint>

// BirdFly G474 超级电容控制器（njupt-robomasters/G474SuperCap）
// 控制器只在收到命令时应答，100 ms 未收到命令关闭 DCDC；任何模式都需由调用方按固定周期持续调用 send()
// DCDC 使能后把电池侧功率稳定在目标值，负载超过目标时电容自动放电
class SuperCap {
  public:
    struct Config {
        Can can;
        uint32_t tx_id = 0x061, rx_id = 0x051;
        float timeout = 0.1f;             // 应答超时，s
        float voltage_cutoff = 10;        // BOOST 放电截止电压，V，电容低于此值回退 CHARGE
        float voltage_resume = 12;        // BOOST 恢复电压，V
        float max_discharge_current = 10; // 最大补偿电流，A，BOOST 功率上限叠加此值·电容电压
        float power_margin = 5;           // 功率裕量，W：电池侧目标与实测控制上限取裁判上限减此值
    };

    enum Mode {
        OFF,     // 关闭 DCDC，不用实测功率
        MONITOR, // 关闭 DCDC，只测功率
        CHARGE,  // 使能 DCDC，功率上限不超过电池侧目标，只充电
        BOOST,   // 使能 DCDC，允许电容放电补偿
    };

    bool is_connect = false;
    Mode mode = OFF;        // 设定模式
    Mode active_mode = OFF; // 生效模式：离线为 OFF，BOOST 电容欠压时为 CHARGE

    float battery_power_ref{}; // 电池侧目标功率，W

    // 应答测量
    struct {
        float out{}; // 底盘口（电池侧）电压，V
        float cap{}; // 电容电压，V
    } voltage;

    struct {
        float out{}; // 底盘负载电流，A
        float cap{}; // 电容电流，A，充电为正
    } current;

    struct {
        float out{}; // 底盘负载功率，W
    } power;

    explicit SuperCap(const Config& config);

    const Config& config() const;

    // 设置模式，下次 update() 生效
    void set_mode(Mode mode);

    // 设置电池侧目标功率，W；发送时限幅到协议范围 [0, 327.67]
    void set_battery_power(float battery_power);

    // 每个控制周期调用：应答超时检测与生效模式
    void update();

    // 发送一帧命令；调用周期由调用方控制，须小于控制器 100 ms 心跳超时
    void send() const;

  private:
    const Config config_;

    Timer rx_timer_;
    bool is_cap_low_ = false; // 电容电压低于截止后未回升到恢复电压

    void can_callback(const CanFrame& frame);
};
