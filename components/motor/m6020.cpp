#include "m6020.hpp"

M6020::M6020(const uint8_t can_port, const uint32_t feedback_can_id) :
    MDJI(can_port, feedback_can_id, CURRENT_MAX, CURRENT_CMD_MAX) {
    SetReduction(REDUCTION);
    SetKt(Kt);
}
