#include "m6020.hpp"

M6020::M6020(uint8_t can_port, uint32_t feedback_can_id) :
    MDJI(can_port, feedback_can_id,
         CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION) {}

void M6020::Update() {
    if (is_enable) {
        const Angle angle_err = angle.ref - angle.measure;
        const float speed_err = speed.ref - speed.measure;
        current.ref = pid.CalcPosition(angle_err, speed_err);
    } else {
        pid.Clear();
        current.ref = 0;
    }
}
