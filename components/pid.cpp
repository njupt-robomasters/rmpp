#include <pid.hpp>
#include <cmath>
#include "bsp_dwt.h"

void PID::Init(const float Kp, const float Ki, const float Kd, const float max_out, const float intergral_limit) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    this->max_out = max_out;
    this->integral_limit = intergral_limit;

    Clear();
}

// 启用梯形积分
void PID::EnableTrapezoidIntergral() {
    improve |= Trapezoid_Intergral;
}

// 启用变速积分
void PID::EnableChangingIntegrationRate(const float CofA, const float CofB) {
    this->coefA = CofA;
    this->coefB = CofB;
    improve |= Changing_Integration_Rate;
}

// 启用微分先行
void PID::EnableDerivativeOnMeasurement() {
    improve |= Derivative_On_Measurement;
}

// 启用微分滤波
void PID::EnableDerivativeFilter(const float derivative_lpf_rc) {
    this->derivative_lpf_rc = derivative_lpf_rc;
    improve |= Derivative_Filter;
}

// 启用输出滤波
void PID::EnableOutputFilter(const float output_lpf_rc) {
    this->output_lpf_rc = output_lpf_rc;
    improve |= Output_Filter;
}

float PID::Calculate(const float measure, const float ref) {
    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    this->measure = measure;
    this->ref = ref;
    err = ref - measure;

    // 基础PID计算
    Pout = Kp * err;
    iterm = Ki * err * dt;
    Dout = Kd * (err - last_err) / dt;

    // 梯形积分
    if (improve & Trapezoid_Intergral)
        f_Trapezoid_Intergral();
    // 变速积分
    if (improve & Changing_Integration_Rate)
        f_Changing_Integration_Rate();
    // 微分先行
    if (improve & Derivative_On_Measurement)
        f_Derivative_On_Measurement();
    // 微分滤波器
    if (improve & Derivative_Filter)
        f_Derivative_Filter();

    // 积分限幅
    f_Integral_Limit();

    Iout += iterm;

    output = Pout + Iout + Dout;

    // 输出滤波
    if (improve & Output_Filter)
        f_Output_Filter();

    // 输出限幅
    if (output > max_out) {
        output = max_out;
    }
    if (output < -max_out) {
        output = -max_out;
    }

    last_err = err;

    return output;
}

void PID::Clear() {
    output = Pout = Iout = Dout = 0;
    last_err = last_measure = last_output = last_Dout = 0;
}

// 积分限幅
void PID::f_Integral_Limit() {
    // 情况一：I输出超过设定最大值 -> 对I输出钳位
    const float temp_Iout = Iout + iterm;
    if (temp_Iout > integral_limit) {
        iterm = 0;
        Iout = integral_limit;
    }
    if (temp_Iout < -integral_limit) {
        iterm = 0;
        Iout = -integral_limit;
    }

    // 情况二：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    const float temp_output = Pout + Iout + Dout;
    if (fabsf(temp_output) > max_out) {
        if (err * Iout > 0) {
            // 积分呈累积趋势
            iterm = 0;
        }
    }
}

// 梯形积分
void PID::f_Trapezoid_Intergral() {
    iterm = Ki * ((err + last_err) / 2) * dt;
}

// 变速积分
void PID::f_Changing_Integration_Rate() {
    if (err * Iout > 0) {
        // 积分呈累积趋势
        if (fabsf(err) <= coefB)
            return; // Full integral
        if (fabsf(err) <= coefA + coefB)
            iterm *= (coefA - fabsf(err) + coefB) / coefA;
        else
            iterm = 0;
    }
}

// 微分先行
void PID::f_Derivative_On_Measurement() {
    Dout = Kd * (last_measure - measure) / dt;
    last_measure = measure;
}

// 微分滤波
void PID::f_Derivative_Filter() {
    Dout = Dout * dt / (derivative_lpf_rc + dt) + last_Dout * derivative_lpf_rc / (derivative_lpf_rc + dt);
    last_Dout = Dout;
}

// 输出滤波
void PID::f_Output_Filter() {
    output = output * dt / (output_lpf_rc + dt) + last_output * output_lpf_rc / (output_lpf_rc + dt);
    last_output = output;
}
