#include "m3508.hpp"
#include "utils.hpp"
#include "bsp_dwt.h"

M3508::M3508(const float &Kp, const float &Kd) : MDJI(MAX_CURRENT, 16384, 3591.0f / 187.0f),
                                                                    Kp(Kp), Kd(Kd) {
}

void M3508::Update(const float v_tps_ref) {
    this->v_tps_ref = v_tps_ref;

    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    // 增量式PID
    Pout = Kp * (v_tps_ref - v_tps_lpf);
    Dout = -Kd * a_tpss;
    current_ref += (Pout + Dout) * dt;

    clamp(current_ref, MAX_CURRENT);
}
