#pragma once

#include "mdji.hpp"
#include "pid.hpp"

class M2006 : public MDJI {
public:
    M2006(uint8_t can_port, uint32_t feedback_can_id);

    void Update();

private:
    static constexpr UnitFloat<A> CURRENT_MAX = 10.0f;
    static constexpr int16_t CURRENT_CMD_MAX = 10000;
    static constexpr float REDUCTION = 36.0f;
};
