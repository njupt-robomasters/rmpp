#include "m6020.hpp"

M6020::M6020(uint8_t can_port, uint32_t feedback_can_id) :
    MDJI(can_port, feedback_can_id,
         CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION) {}

void M6020::SetCircular(bool is_circular, const Angle<>& mid) {
    this->is_circular = is_circular;
    if (!is_circular) {
        this->mid = mid;
    }
}

void M6020::Update() {
    if (is_enable) {
        if (is_circular) {
            const Angle angle_err = angle.ref - angle.measure;
            const float speed_err = speed.ref - speed.measure;
            current.ref = pid.CalcMIT(angle_err, speed_err);
        } else {
            const Angle ref_relative = angle.ref - mid;
            const Angle measure_relative = angle.measure - mid;
            const UnitFloat angle_err = ref_relative - measure_relative;
            const float speed_err = speed.ref - speed.measure;
            current.ref = pid.CalcMIT(angle_err, speed_err);
        }
    }
}
