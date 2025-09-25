#pragma once

#include "mdji.hpp"

class M6020 : public MDJI {
public:
    M6020(uint8_t can_port, uint32_t feedback_can_id);

    void SetCircular(bool is_circular, const Angle<>& mid = 0);

    void Update();

private:
    static constexpr UnitFloat<A> CURRENT_MAX = 3.0f * A;
    static constexpr int16_t CURRENT_CMD_MAX = 16384;
    static constexpr float REDUCTION = 1.0f;

    bool is_circular = true;
    Angle<deg> mid;
};
