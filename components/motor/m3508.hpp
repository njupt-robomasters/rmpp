#pragma once

#include "mdji.hpp"

class M3508 : public MDJI {
public:
    static constexpr float MAX_CURRENT = 20.0f; // 最大电流【单位；A】

    float v_tps_ref = 0; // 目标速度（减速后）【单位：rpm】

    M3508(const float &Kp, const float &Kd);

    void Update(float v_tps_ref);

private:
    const float &Kp, &Kd;

    float Pout = 0, Dout = 0;

    // 用于增量式PID
    uint32_t dwt_cnt = 0;
    float dt = 0;
};
