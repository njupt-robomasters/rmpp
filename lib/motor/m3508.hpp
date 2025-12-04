#pragma once

#include "mdji.hpp"

class M3508 : public MDJI {
public:
    static constexpr UnitFloat CURRENT_MAX = 20.0f * A;
    static constexpr uint16_t CURRENT_CMD_MAX = 16384;
    static constexpr float REDUCTION = 3591.0f / 187.0f;
    static constexpr UnitFloat Kt = 0.3f * Nm_A;

    // 电流衰减系数，用于功率控制
    UnitFloat<pct> current_ratio = 100.0f * pct;

    M3508(uint8_t can_port, uint32_t feedback_can_id);

    // 设置电流衰减，用于功率控制（todo: 解耦功率控制）
    void SetCurrentRatio(const UnitFloat<>& current_ratio);

    int16_t GetCurrentCMD() const;
};
