#pragma once

#include <cstdint>

class PID {
public:
    // PID输出
    float output = 0;

    void Init(float Kp, float Ki, float Kd, float max_out, float intergral_limit);

    void EnableTrapezoidIntergral();

    void EnableChangingIntegrationRate(float CofA, float CofB);

    void EnableDerivativeOnMeasurement();

    void EnableDerivativeFilter(float derivative_lpf_rc);

    void EnableOutputFilter(float output_lpf_rc);

    float Calculate(float measure, float ref);

    void Clear();

private:
    enum improve_e {
        NONE = 0x00,
        Trapezoid_Intergral = 1 << 0, // 梯形积分
        Changing_Integration_Rate = 1 << 1, // 变速积分
        Derivative_On_Measurement = 1 << 2, // 微分先行
        Derivative_Filter = 1 << 3, // 微分滤波
        Output_Filter = 1 << 4 // 输出滤波
    };

    // PID参数
    float Kp = 0, Ki = 0, Kd = 0; // P、I、D三参数
    float max_out = 0; // 输出限幅
    float integral_limit = 0; // 积分限幅
    float coefA = 0, coefB = 0; // 变速积分参数，iterm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float output_lpf_rc = 0, derivative_lpf_rc = 0; // 输出滤波、积分滤波，RC = 1/omegac

    // 测量值、参考值、误差值
    float measure = 0, ref = 0, err = 0;

    float Pout = 0, Iout = 0, Dout = 0;

    float iterm = 0;

    float last_err = 0; // 用于梯形积分
    float last_measure = 0; // 用于微分先行
    float last_output = 0; // 用于输出滤波
    float last_Dout = 0; // 用于微分滤波

    uint32_t dwt_cnt = 0;
    float dt = 0;

    uint8_t improve = NONE;

    void f_Integral_Limit();

    // PID优化函数
    void f_Trapezoid_Intergral(); // 梯形积分

    void f_Changing_Integration_Rate(); // 变速积分

    void f_Derivative_On_Measurement(); // 微分先行

    void f_Derivative_Filter(); // 微分滤波

    void f_Output_Filter(); // 输出滤波
};
