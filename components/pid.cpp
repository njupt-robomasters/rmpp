#include <pid.hpp>
#include <cmath>
#include "utils.hpp"

#define kp (param.kp)
#define ki (param.ki)
#define kd (param.kd)
#define ff (param.ff)
#define max_i (param.max_i)
#define max_out (param.max_out)

PID::PID(param_t& param) : param(param) {}

void PID::SetParam(param_t& param) {
    this->param = param;
}

// 经典pid
float PID::CalcPosition(const float err) {
    this->err = err;

    dt = dwt.GetDT();

    // 基础PID计算
    p_out = kp * err;
    di = ki * err * dt;
    d_out = kd * (err - last_err) / dt;

    calcPositionCommon();

    return out;
}

// mit控制模式
float PID::CalcPosition(const float err, const float derr) {
    this->err = err;

    dt = dwt.GetDT();

    // 基础PID计算
    p_out = kp * err;
    di = ki * err * dt;
    d_out = kd * derr;

    calcPositionCommon();

    return out;
}

// 增量式pid
float PID::CalcIncrement(const float err) {
    this->err = err;

    dt = dwt.GetDT();

    p_out = kp * (err - last_err);
    i_out = ki * err;
    d_out = kd * (err - 2 * last_err + last2_err);

    // 积分限幅：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (fabsf(out + (p_out + i_out + d_out) * dt) >= max_out) {
        if (err * i_out > 0) { // 积分呈累积趋势
            i_out = 0;
        }
    }

    out_without_ff += (p_out + i_out + d_out) * dt;
    out = out_without_ff + ff;

    // 输出限幅
    out = clamp(out, max_out);

    last2_err = last_err;
    last_err = err;

    return out;
}

void PID::Clear() {
    err = 0;
    p_out = 0, i_out = 0, d_out = 0, out_without_ff = 0;
    last_err = 0;
    last2_err = 0;
    di = 0;

    dwt.Clear();
    dt = 0;
}

void PID::calcPositionCommon() {
    // 积分限幅
    // 情况一：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (fabsf(p_out + i_out + d_out) >= max_out) {
        if (err * i_out > 0) { // 积分呈累积趋势
            di = 0;
        }
    }
    // 情况二：I输出超过设定最大值 -> 对I输出钳位
    i_out += di;
    i_out = clamp(i_out, max_i);

    out_without_ff = p_out + i_out + d_out;
    out = out_without_ff + ff;

    // 输出限幅
    out = clamp(out, max_out);

    last_err = err; // 用于下一次求d输出
}
