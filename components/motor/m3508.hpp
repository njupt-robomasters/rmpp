#pragma once

#include "mdji.hpp"
#include "utils.hpp"

class M3508 : public MDJI {
public:
    static constexpr float CURRENT_MAX = 20.0f; // 最大电流【单位；A】

    bool is_enable = false;
    float v_tps_ref = 0; // 目标速度（减速后）【单位：rpm】

    explicit M3508(const mit_t &mit);

    void SetEnable(bool is_enable);

    void SetV_TPS(float v_tps_ref);

    void Update();

private:
    // MIT格式控制参数
    const mit_t &mit;

    float Pout = 0, Dout = 0;

    // 用于增量式PD控制
    uint32_t dwt_cnt = 0;
    float dt = 0;
};
