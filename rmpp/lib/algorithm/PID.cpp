#include "PID.hpp"

#define mode (config->mode)
#define kp (config->kp)
#define ki (config->ki)
#define kd (config->kd)
#define ff (config->ff)
#define max_i (config->max_i)
#define max_out (config->max_out)
#define deadline (config->deadline)
#define fc (config->fc)

PID::PID() : config(nullptr) {}

PID::PID(const config_t* config) : config(config) {}

UnitFloat<> PID::Calculate(const UnitFloat<>& err, std::optional<UnitFloat<>> derr) {
    if (config == nullptr) {
        return 0 * default_unit;
    }

    // 位置PID or 增量PID
    if (mode == POSITION_MODE) {
        calcPosition(err, derr);
    } else if (mode == INCREMENT_MODE) {
        calcIncrement(err);
    }

    return out;
}

void PID::calcPosition(const UnitFloat<>& err, const std::optional<UnitFloat<>>& derr) {
    // 计算dt
    const UnitFloat dt = dwt.UpdateDT();

    // 输入死区
    if (unit::abs(err) < deadline) {
        this->err = 0 * default_unit;
    } else {
        this->err = err;
    }

    // 基础PID计算
    p_out = kp * this->err;
    UnitFloat<> di = ki * this->err * dt;
    if (derr.has_value()) {
        d_out = kd * derr.value();
    } else {
        d_out = kd * (this->err - last_err) / dt;
    }

    // 积分限幅
    // 情况一：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (unit::abs(out) >= max_out) {
        if (di * i_out > 0) { // 积分呈累积趋势
            di = 0 * default_unit;
        }
    }
    // 情况二：I输出超过设定最大值 -> 对I输出钳位
    i_out += di;
    i_out = unit::clamp(i_out, max_i);

    // 输出滤波
    out_no_filter = p_out + i_out + d_out + ff;
    out = lowpassFilter(out, out_no_filter, fc, dt);

    // 输出限幅
    out = unit::clamp(out, max_out);

    last_err = this->err;
}

void PID::calcIncrement(const UnitFloat<>& err) {
    // 计算dt
    const UnitFloat dt = dwt.UpdateDT();

    // 输入死区
    if (unit::abs(err) < deadline) {
        this->err = 0 * default_unit;
    } else {
        this->err = err;
    }

    // 基础PID计算
    p_out = kp * (this->err - last_err);
    i_out = ki * this->err * dt;
    d_out = kd * (this->err - 2 * last_err + last_err2) / dt;

    // 积分限幅：总输出饱和，阻止I继续累积
    if (unit::abs(out) >= max_out) {
        i_out = 0 * default_unit;
    }

    // 输出滤波
    out_no_filter += p_out + i_out + d_out;
    out = lowpassFilter(out, out_no_filter, fc, dt);

    // 输出限幅
    out = unit::clamp(out, max_out);

    last_err2 = last_err;
    last_err = this->err;
}

void PID::Clear() {
    err = last_err = last_err2 = 0 * default_unit;
    out = out_no_filter = p_out = i_out = d_out = 0 * default_unit;
    dwt.Reset();
}

#undef fc

UnitFloat<> PID::lowpassFilter(const UnitFloat<>& pre, const UnitFloat<>& next, const UnitFloat<>& fc, const UnitFloat<>& dt) {
    if (fc == 0) { // fc==0，禁用滤波
        return next;
    }
    const UnitFloat rc = 1 / (2 * PI * fc);
    const UnitFloat alpha = dt / (rc + dt);
    const UnitFloat out = pre * (1 * default_unit - alpha) + next * alpha;
    return out;
}