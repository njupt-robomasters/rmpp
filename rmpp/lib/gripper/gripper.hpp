#pragma once

#include "bsp/bsp.hpp"

/**
 * @brief 夹爪控制类
 * 
 * 通过CAN总线控制夹爪的松开和夹紧动作
 * CAN协议格式：
 * - data[0]: 状态（0=松开，1=夹紧）
 * - data[1]: 最大速度（度/秒）
 * - data[2]: 电流高位（毫安）
 * - data[3]: 电流低位（毫安）
 * - data[4-7]: 保留（0x00）
 */
class Gripper {
public:
    struct config_t {
        uint8_t can_port = 1;
        uint32_t cmd_id = 0x20;
        uint32_t feedback_id = 0x21;
        UnitFloat<> timeout = 100.0f * ms;
    } config;

    bool is_connect = false;
    bool is_enable = false;
    bool is_close = false;

    UnitFloat<deg_s> speed;

    struct {
        UnitFloat<A> ref, measure1, measure2;
    } current;

    Gripper(const config_t& config);

    void SetEnable(bool is_enable);

    // 松开夹爪
    void Open(const UnitFloat<>& speed);

    // 夹紧夹爪
    void Close(const UnitFloat<>& speed, const UnitFloat<>& current);

    void SendCanCmd();

    // 需要在循环中调用
    void OnLoop();

private:
    BSP::Dwt dwt_connect;

    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};
