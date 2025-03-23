#include <pid.hpp>
#include <cmath>
#include "bsp_dwt.h"
#include "utils.hpp"

#define kp (param.kp)
#define ki (param.ki)
#define kd (param.kd)
#define ff (param.ff)
#define i_limit (param.i_limit.value())
#define max_out (param.max_out.value())

// 使用参数结构体初始化
PID::PID(param_t &param) : param(param.SetDefault()) {
}

// // 启用梯形积分
// void PID::EnableTrapezoidIntergral() {
//     improve |= Trapezoid_Intergral;
// }
//
// // 启用变速积分
// void PID::EnableChangingIntegrationRate(const float cofA, const float cofB) {
//     this->coefA = cofA;
//     this->coefB = cofB;
//     improve |= Changing_Integration_Rate;
// }
//
// // 启用微分滤波
// void PID::EnableDerivativeFilter(const float derivative_lpf_rc) {
//     this->derivative_lpf_rc = derivative_lpf_rc;
//     improve |= Derivative_Filter;
// }
//
// // 启用输出滤波
// void PID::EnableOutputFilter(const float output_lpf_rc) {
//     this->output_lpf_rc = output_lpf_rc;
//     improve |= Output_Filter;
// }

// err = ref - measure
float PID::CalcPosition(const float err) {
    this->err = err;

    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    // 基础PID计算
    Pout = kp * err;
    iterm = ki * err * dt;
    Dout = kd * (err - last_err) / dt;

    calcPositionCommon();

    return out;
}

// err = ref - measure
float PID::CalcPosition(const float err, const float dset) {
    this->err = err;

    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    // 基础PID计算
    Pout = kp * err;
    iterm = ki * err * dt;
    Dout = kd * dset;

    calcPositionCommon();

    return out;
}

float PID::CalcIncrement(const float err) {
    this->err = err;

    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    Pout = kp * (err - last_err);
    Iout = ki * err;
    Dout = kd * (err - 2 * last_err + last2_err);

    // 积分限幅：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (fabsf(out + (Pout + Iout + Dout) * dt) >= max_out) {
        if (err * Iout > 0) {
            // 积分呈累积趋势
            Iout = 0;
        }
    }

    out_without_ff += (Pout + Iout + Dout) * dt;
    out = out_without_ff + ff;

    // 输出限幅
    out = clamp(out, max_out);

    last2_err = last_err;
    last_err = err;

    return out;
}

void PID::Clear() {
    err = 0;
    Pout = 0, Iout = 0, Dout = 0, out_without_ff = 0;
    last_err = 0;
    last2_err = 0;
    // last_out = 0;
    // last_Dout = 0;
    iterm = 0;
    dwt_cnt = 0;
    dt = 0;
}

void PID::calcPositionCommon() {
    // // 梯形积分
    // if (improve & Trapezoid_Intergral)
    //     f_Trapezoid_Intergral();
    // // 变速积分
    // if (improve & Changing_Integration_Rate)
    //     f_Changing_Integration_Rate();
    // // 微分滤波器
    // if (improve & Derivative_Filter)
    //     f_Derivative_Filter();

    // 积分限幅
    // 情况一：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (fabsf(Pout + Iout + Dout) >= max_out) {
        if (err * Iout > 0) {
            // 积分呈累积趋势
            iterm = 0;
        }
    }
    // 情况二：I输出超过设定最大值 -> 对I输出钳位
    Iout += iterm;
    Iout = clamp(Iout, i_limit);

    out_without_ff = Pout + Iout + Dout;
    out = out_without_ff + ff;

    // // 输出滤波
    // if (improve & Output_Filter)
    //     f_Output_Filter();

    // 输出限幅
    out = clamp(out, max_out);

    last_err = err; // 用于求Dout和梯形积分
}

// // 梯形积分
// void PID::f_Trapezoid_Intergral() {
//     iterm = ki * ((err + last_err) / 2) * dt;
// }
//
// // 变速积分
// void PID::f_Changing_Integration_Rate() {
//     if (err * Iout > 0) {
//         // 积分呈累积趋势
//         if (fabsf(err) <= coefB)
//             return; // Full integral
//         if (fabsf(err) <= coefA + coefB)
//             iterm *= (coefA - fabsf(err) + coefB) / coefA;
//         else
//             iterm = 0;
//     }
// }
//
// // 微分滤波
// void PID::f_Derivative_Filter() {
//     Dout = Dout * dt / (derivative_lpf_rc + dt) + last_Dout * derivative_lpf_rc / (derivative_lpf_rc + dt);
//     last_Dout = Dout;
// }
//
// // 输出滤波
// void PID::f_Output_Filter() {
//     out = out * dt / (output_lpf_rc + dt) + last_output * output_lpf_rc / (output_lpf_rc + dt);
//     last_output = out;
// }
