#pragma once

#include "unit.hpp"
#include "pid.hpp"

class N630 {
private:
    static constexpr uint8_t POLES = 14; // 电机磁极数

    typedef enum {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT = 1,
        CAN_PACKET_SET_CURRENT_BRAKE = 2,
        CAN_PACKET_SET_RPM = 3,
        CAN_PACKET_SET_POS = 4,
        CAN_PACKET_SET_CURRENT_REL = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
        CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13
    } oommand_e;

    typedef enum {
        CAN_PACKET_STATUS = 9,
        CAN_PACKET_STATUS_2 = 14,
        CAN_PACKET_STATUS_3 = 15,
        CAN_PACKET_STATUS_4 = 16,
        CAN_PACKET_STATUS_5 = 27,
        CAN_PACKET_STATUS_6 = 58
    } status_e;

    const uint8_t can_port;
    const uint32_t n630_id;

    bool is_enable = false;

    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    void makeCANData(uint8_t buffer[4], float value, float scale);

public:
    // status 1
    struct {
        UnitFloat<rpm> ref, measure;
        uint32_t raw;
    } speed;

    UnitFloat<A> current;
    UnitFloat<percentage> duty;

    // status 4
    UnitFloat<C> temperate_mos;
    UnitFloat<C> temperate_motor;
    UnitFloat<A> current_in;

    N630(uint8_t can_port, uint32_t n630_id);

    void SetEnable(bool is_enable);

    void SetSpeed(const UnitFloat<>& speed);

    void Update();
};
