#pragma once

#include "mdji.hpp"

class M2006 : public MDJI {
public:
    static constexpr UnitFloat CURRENT_MAX = 10.0f * A;
    static constexpr uint16_t CURRENT_CMD_MAX = 10000;
    static constexpr float REDUCTION = 36.0f;
    static constexpr UnitFloat Kt = 0.18f * Nm_A;

    M2006(uint8_t can_port, uint32_t feedback_can_id);
};
