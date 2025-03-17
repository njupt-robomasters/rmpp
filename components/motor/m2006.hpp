#pragma once

#include "mdji.hpp"

class M2006 : public MDJI {
public:
    static constexpr float MAX_CURRENT = 10.0f; // 最大电流【单位；A】

    float speed_tps_ref = 0; // 目标速度（减速后）【单位：圈/s】

    M2006(float Kp, float Ki, float Imax);

    void Update(float speed_tps_ref);

private:
    float Kp, Ki, Imax;
};
