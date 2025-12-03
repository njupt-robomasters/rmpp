#include "m3508.hpp"

M3508::M3508(const uint8_t can_port, const uint32_t feedback_can_id) :
    MDJI(can_port, feedback_can_id, CURRENT_MAX, CURRENT_CMD_MAX) {
    SetReduction(REDUCTION);
    SetKt(Kt);
}

void M3508::SetCurrentRatio(const UnitFloat<>& current_ratio) {
    this->current_ratio = current_ratio;
}

int16_t M3508::GetCurrentCMD() const {
    int16_t current_cmd;
    if (!is_invert) {
        current_cmd = (int16_t)((current.ref * current_ratio / CURRENT_MAX).toFloat(A) * (float)CURRENT_CMD_MAX);
    } else {
        current_cmd = (int16_t)((-current.ref * current_ratio / CURRENT_MAX).toFloat(A) * (float)CURRENT_CMD_MAX);
    }
    return current_cmd;
}
