#pragma once

#include "mdji.hpp"

class M6020 : public MDJI {
public:
    static constexpr auto MAX_CURRENT = 3.0f; // 最大电流【单位；A】

    // 目标角度【单位：角度】
    float angle_ref = 0;

    M6020(float Kp, float Kd, float feedforward);

    void Update(float angle_ref);

private:
    // MIT控制参数
    float Kp, Kd, feedforward ;

    float Pout = 0, Dout = 0;
};
