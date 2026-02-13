#pragma once

#include "bsp/bsp.hpp"

class UI {
public:
    struct config_t {
        // CAN通信参数
        uint8_t can_port = 1;
        uint32_t master_id = 0;
        uint32_t slave_id = 1;

        // 最大值
        UnitFloat<> max_cap_ratio = 1 * ratio;
        UnitFloat<> max_chassis_wr = 120 * rpm;
        UnitFloat<> max_shoot_freq = 10 * Hz;
        UnitFloat<> max_shoot_current = 10 * A;
    } config;

    // 云台相对底盘角度
    Angle<deg> yaw_ecd;

    // 云台角度
    Angle<deg> yaw, pitch;

    // 伤害方向
    bool is_hurt = false;
    Angle<deg> hurt_dir;

    // 电容剩余能量比例
    UnitFloat<ratio> cap_ratio;

    // 底盘旋转速度
    UnitFloat<rpm> chassis_wr;

    // 弹频
    UnitFloat<Hz> shoot_freq;

    // 拨弹电机电流
    UnitFloat<A> shoot_current;

    // 自瞄状态
    bool is_detect = false;

    UI(const config_t& config);

    static void AddCanData(const uint8_t data[], size_t len);

    void Init();

    // 需要在循环中调用
    void OnLoop();

    void SendCanCmd();

private:
    // CAN发送队列
    static constexpr size_t TXBUF_SIZE = 1024;
    static uint8_t txbuf[TXBUF_SIZE];
    static size_t head, tail;

    // 发送频率
    static constexpr UnitFloat<> INIT_FREQ = 15 * Hz;
    static constexpr UnitFloat<> UPDATE_FREQ = 30 * Hz;

    enum {
        INIT,
        UPDATE
    } state = INIT;

    uint8_t init_index = 0;
    uint32_t update_index = 0;

    BSP::Dwt dwt; // 用于控制发送频率

    // 更新UI库
    void updateLib() const;
};