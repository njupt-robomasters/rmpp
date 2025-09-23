#pragma once

#include "unit.hpp"
#include "pid.hpp"

class N630 {
private:
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
    } oommand_id_e;

    const uint8_t can_port;
    const uint32_t n630_id;

    bool is_enable = false;

    UnitFloat<rpm> speed;

    void makeCANData(uint8_t buffer[4], float value, float scale);

public:
    N630(uint8_t can_port, uint32_t n630_id);

    void SetEnable(bool is_enable);

    void SetSpeed(UnitFloat<rpm> speed);

    void Update();
};
