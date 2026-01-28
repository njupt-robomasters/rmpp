#pragma once

#include "bsp/bsp.hpp"

class UI {
public:
    struct config_t {
        // CAN通信参数
        uint8_t can_port = 1;
        uint32_t master_id = 0;
        uint32_t slave_id = 1;
        UnitFloat<> update_freq = 10.0f * Hz;
    } config;

    // 云台yaw角
    Angle<deg> yaw;

    // 伤害提示
    bool is_hurt = false;
    Angle<deg> hurt_dir;

    // 摩擦轮速度
    UnitFloat<m_s> bullet_speed_1;
    UnitFloat<m_s> bullet_speed_2;

    // 拨弹电机电流
    UnitFloat<A> shoot_current;

    // 自瞄状态
    struct {
        bool is_connect = false;
        bool is_detect = false;
    } aim;

    UI(const config_t& config);

    static void AddTxData(const uint8_t data[], size_t len);

    void Init();

    void OnLoop();

private:
    static constexpr size_t TXBUF_SIZE = 1024;

    uint8_t state = 0;

    BSP::Dwt dwt_update;

    // CAN发送队列
    static uint8_t txbuf[TXBUF_SIZE];
    static size_t head, tail;

    void handleLib();

    void handleTx();
};