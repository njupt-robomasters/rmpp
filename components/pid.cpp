#include "pid.hpp"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID::Init(PID_MODE mode_, float Kp_, float Ki_, float Kd_, float max_out_, float max_Iout_) {
    mode = mode_;
    Kp = Kp_, Ki = Ki_, Kd = Kd_;
    max_out = max_out_, max_Iout = max_Iout_;
    now = set = 0.0f;
    out = Pout = Iout = Dout = 0.0f;
    error[0] = error[1] = error[2] = 0.0f;
    Dbuf[0] = Dbuf[1] = Dbuf[2] = 0.0f;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      now: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
float PID::Update(float now_, float set_) {
    now = now_;
    set = set_;

    error[2] = error[1];
    error[1] = error[0];
    error[0] = set - now;

    if (mode == PID_POSITION) {
        Pout = Kp * error[0];
        Iout += Ki * error[0];
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = (error[0] - error[1]);
        Dout = Kd * Dbuf[0];
        LimitMax(Iout, max_Iout);
        out = Pout + Iout + Dout;
        LimitMax(out, max_out);
    } else if (mode == PID_DELTA) {
        Pout = Kp * (error[0] - error[1]);
        Iout = Ki * error[0];
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = error[0] - 2.0f * error[1] + error[2];
        Dout = Kd * Dbuf[0];
        out += Pout + Iout + Dout;
        LimitMax(out, max_out);
    }
    return out;
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID::Clear() {
    now = set = 0.0f;
    out = Pout = Iout = Dout = 0.0f;
    error[0] = error[1] = error[2] = 0.0f;
    Dbuf[0] = Dbuf[1] = Dbuf[2] = 0.0f;
}
