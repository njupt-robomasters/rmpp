#pragma once

#include "mdji.hpp"
#include "utils.hpp"

class M6020 : public MDJI {
public:
    static constexpr float CURRENT_MAX = 3.0f; // 最大电流【单位；A】

    bool is_enable = false;
    float angle_ref = 0; // 目标角度【单位：角度】
    float v_aps_ff = 0; // 速度前馈

    explicit M6020(const mit_t &mit);

    void SetEnable(bool is_enable);

    void SetAngle(float angle_ref);

    void SetV_APS_FF(float v_aps_ff);

    void Update();

private:
    // MIT格式控制参数
    const mit_t &mit;

    float Pout = 0, Dout = 0;
};
