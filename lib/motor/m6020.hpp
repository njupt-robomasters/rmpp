#pragma once

#include "mdji.hpp"

class M6020 : public MDJI {
public:
    static constexpr UnitFloat CURRENT_MAX = 3.0f * A;
    static constexpr uint16_t CURRENT_CMD_MAX = 16384;
    static constexpr float REDUCTION = 1.0f;
    static constexpr UnitFloat Kt = 0.741f * Nm_A;

    M6020(uint8_t can_port, uint32_t feedback_can_id);
};
