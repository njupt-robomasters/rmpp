#pragma once

#include "mdji.hpp"
#include "pid.hpp"

class M6020 : public MDJI {
public:
    M6020(uint8_t can_port, uint32_t feedback_can_id);

    void Update();

private:
    static constexpr UnitFloat<A> CURRENT_MAX = 3.0f;
    static constexpr int16_t CURRENT_CMD_MAX = 16384;
    static constexpr float REDUCTION = 1.0f;
};
