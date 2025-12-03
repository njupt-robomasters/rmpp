#include <pid.hpp>

#define kp (param->kp)
#define ki (param->ki)
#define kd (param->kd)
#define ff (param->ff)
#define max_i (param->max_i)
#define max_out (param->max_out)
#define fc (param->fc)

PID::PID(param_t* param) : param(param) {}

void PID::SetParam(param_t* param) {
    this->param = param;
}

UnitFloat<> PID::Calculate(const UnitFloat<>& err, std::optional<UnitFloat<>> derr) {
    if (param == nullptr) {
        return 0 * default_unit;
    }

    if (param->mode == POSITION_MODE) {
        calcPosition(err, derr);
    } else if (param->mode == INCREMENT_MODE) {
        calcIncrement(err);
    }

    return out;
}

void PID::calcPosition(const UnitFloat<>& err, const std::optional<UnitFloat<>>& derr) {
    // 计算dt
    const float dt = dwt.GetDT();

    // 输入滤波
    this->err = lowpassFilter(this->err, err, fc, dt);

    // 基础PID计算
    p_out = kp * err;
    di = ki * err * dt;
    if (derr.has_value()) {
        d_out = kd * derr.value();
    } else {
        d_out = kd * (err - last_err) / dt;
    }

    // 积分限幅
    // 情况一：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (unit::abs(p_out + i_out + d_out) >= max_out) {
        if (di * i_out > 0) { // 积分呈累积趋势
            di = 0 * default_unit;
        }
    }
    // 情况二：I输出超过设定最大值 -> 对I输出钳位
    i_out += di;
    i_out = unit::clamp(i_out, max_i);

    // 输出和限幅
    out = p_out + i_out + d_out + ff;
    out = unit::clamp(out, max_out);

    last_err = err;
}

void PID::calcIncrement(const UnitFloat<>& err) {
    // 计算dt
    const float dt = dwt.GetDT();

    // 输入滤波
    this->err = lowpassFilter(this->err, err, fc, dt);

    // 基础PID计算
    p_out = kp * (err - last_err);
    i_out = ki * err * dt;
    d_out = kd * (err - 2 * last_err + last_err2) / dt;

    // 积分限幅：总输出饱和，阻止I继续累积
    if (unit::abs(out) >= max_out) {
        i_out = 0 * default_unit;
    }

    // 输出和限幅
    out += p_out + i_out + d_out;
    out = unit::clamp(out, max_out);

    last_err2 = last_err;
    last_err = err;
}

void PID::Clear() {
    err = last_err = last_err2 = 0 * default_unit;
    di = 0 * default_unit;
    out = p_out = i_out = d_out = 0 * default_unit;
    dwt.Clear();
}

#undef kp
#undef ki
#undef kd
#undef ff
#undef max_i
#undef max_out
#undef fc

UnitFloat<> PID::lowpassFilter(const UnitFloat<>& pre, const UnitFloat<>& next, const UnitFloat<>& fc, const float dt) {
    if (fc == 0) { // fc==0，禁用滤波
        return next;
    }
    const float fs = 1 / dt;
    const float alpha = 2 * PI * fc.toFloat() / (fs + 2 * PI * fc.toFloat());
    const UnitFloat out = pre * (1 - alpha) + next * alpha;
    return out;
}
