#include "mdji.hpp"
#include "bsp.hpp"
#include "utils.hpp"

MDJI::MDJI(uint8_t can_port, uint32_t feedback_can_id,
           Unit<A> current_max, uint16_t current_cmd_max, float reduction) :
    can_port(can_port), feedback_can_id(feedback_can_id),
    current_max(current_max), current_cmd_max(current_cmd_max), reduction(reduction) {
    BSP::CAN::RegisterCallback(std::bind(&MDJI::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
}

void MDJI::callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc) {
    if (port != can_port) return;
    if (id != feedback_can_id) return;
    if (dlc != 8) return;

    dt = dwt.GetDT();

    // 记录CAN反馈报文频率
    can_feedback_freq = 1 / dt;

    // 解析CAN报文
    if (!is_invert) {
        angle.raw = (uint16_t)(data[0] << 8 | data[1]);
        speed.raw = (int16_t)(data[2] << 8 | data[3]);
        current.raw = (int16_t)(data[4] << 8 | data[5]);
        temperate = data[6];
    } else {
        angle.raw = 8191 - (uint16_t)(data[0] << 8 | data[1]);
        speed.raw = -(int16_t)(data[2] << 8 | data[3]);
        current.raw = -(int16_t)(data[4] << 8 | data[5]);
        temperate = data[6];
    }

    // 转换为标准单位
    // 电流
    current.measure_no_lfp = (float)current.raw / (float)current_cmd_max * current_max;
    // 低通滤波
    const float alpha1 = solve_alpha(CURRENT_LPF_FREQ, 1 / dt);
    current.measure = lowpass_filter(current.measure, current.measure_no_lfp, alpha1);

    // 角度
    angle.measure = (float)angle.raw / 8192.0f * 360.0f * deg;

    // 转速
    speed.measure_no_lfp = (float)speed.raw * rpm / reduction;
    // 低通滤波
    const float alpha2 = solve_alpha(SPEED_LPF_FREQ, 1 / dt);
    speed.measure = lowpass_filter(speed.measure, speed.measure_no_lfp, alpha2);

    // 电机就绪标识
    is_ready = true;
}

void MDJI::SetInvert(bool is_invert) {
    this->is_invert = is_invert;
}

void MDJI::SetReduction(float reduction) {
    this->reduction = reduction;
}

void MDJI::SetPIDParam(PID::param_t& pid_param) {
    pid.SetParam(pid_param);
}

void MDJI::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) // 避免重复设置
        return;

    this->is_enable = is_enable;

    current.ref = 0.0f;
    pid.Clear();
}

void MDJI::SetAngle(Angle<deg> angle) {
    this->angle.ref = angle;
}

void MDJI::SetSpeed(Unit<m_s> speed) {
    this->speed.ref = speed;
}

int16_t MDJI::GetCurrentCMD() const {
    int16_t current_cmd;
    if (!is_invert) {
        current_cmd = (int16_t)(current.ref / current_max * (float)current_cmd_max);
    } else {
        current_cmd = -(int16_t)(current.ref / current_max * (float)current_cmd_max);
    }
    return current_cmd;
}
