#pragma once

#include "mdji.hpp"
#include "utils.hpp"

class M2006 : public MDJI {
public:
    static constexpr float MAX_CURRENT = 10.0f; // 最大电流【单位；A】

    bool is_enable = false;
    float v_tps_ref = 0; // 目标速度（减速后）【单位：圈/s】

    explicit M2006(const mit_t &mit);

    void SetEnable(bool is_enable);

    void SetV_TPS(float v_tps_ref);

    void Update();

private:
    // MIT格式控制参数
    const mit_t &mit;
};
