#pragma once

#include "mdji.hpp"

class M3508 : public MDJI {
public:
    static constexpr float MAX_CURRENT = 20.0f; // 最大电流【单位；A】

    float speed_rpm_ref = 0; // 目标速度（减速后）【单位：rpm】

    M3508(float Kp, float Ki, float Imax);

    void Update(float speed_rpm_ref);

private:
    float Kp, Ki, Imax;
};
