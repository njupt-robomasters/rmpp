#include "m2006.hpp"

M2006::M2006(uint8_t can_port, uint32_t feedback_can_id) :
    MDJI(can_port, feedback_can_id,
         CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION) {}

void M2006::Update() {
    if (is_enable) {
        const float speed_err = speed.ref - speed.measure;
        current.ref = pid.CalcPosition(speed_err);
    }
}
