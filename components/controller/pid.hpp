#pragma once

#include <cstdint>

class PID {
public:
    struct pid_param_t {
        const float kp = 0, ki = 0, kd = 0, ff = 0; // ff为前馈
        const float i_limit = 0;
    };

    float out = 0;

    // 使用参数结构体初始化
    PID(const pid_param_t &pid_param, float max_out);

    // 使用展开的参数初始化
    PID(const float &kp, const float &ki, const float &kd, const float &ff, const float &i_limit, float max_out);

    // // 启用梯形积分
    // void EnableTrapezoidIntergral();
    //
    // // 启用变速积分
    // void EnableChangingIntegrationRate(float cofA, float cofB);
    //
    // // 启用微分滤波
    // void EnableDerivativeFilter(float derivative_lpf_rc);
    //
    // // 启用输出滤波
    // void EnableOutputFilter(float output_lpf_rc);

    float CalcPosition(float err);

    float CalcPosition(float err, float dset);

    float CalcIncrement(float err);

    void Clear();

private:
    // enum improve_e {
    //     NONE = 0x00,
    //     Trapezoid_Intergral = 1 << 0, // 梯形积分
    //     Changing_Integration_Rate = 1 << 1, // 变速积分
    //     Derivative_Filter = 1 << 2, // 微分滤波
    //     Output_Filter = 1 << 3 // 输出滤波
    // };

    // PID参数
    // 基础参数
    const float &kp, &ki, &kd; // P、I、D三参数
    const float &ff; // 前馈
    const float &i_limit; // 积分限幅
    const float max_out; // 输出限幅
    // 高级参数
    // uint8_t improve = NONE;
    // float coefA = 0, coefB = 0; // 变速积分参数，iterm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    // float output_lpf_rc = 0, derivative_lpf_rc = 0; // 输出滤波、积分滤波，RC = 1/omegac

    // 下面的参数在CLear时需要清空
    float err = 0; // 误差值 err = ref - measure
    float Pout = 0, Iout = 0, Dout = 0, out_without_ff = 0; // PID输出分量
    float last_err = 0; // 用于微分计算、梯形积分
    float last2_err = 0; // 用于增量PID
    float last_out = 0; // 用于输出滤波
    float last_Dout = 0; // 用于微分滤波
    float iterm = 0; // 中间变量
    // 用于维护dt的参数
    uint32_t dwt_cnt = 0;
    float dt = 0;

    void calcPositionCommon();

    // // PID优化函数
    // void f_Trapezoid_Intergral(); // 梯形积分
    //
    // void f_Changing_Integration_Rate(); // 变速积分
    //
    // void f_Derivative_On_Measurement(); // 微分先行
    //
    // void f_Derivative_Filter(); // 微分滤波
    //
    // void f_Output_Filter(); // 输出滤波
};
