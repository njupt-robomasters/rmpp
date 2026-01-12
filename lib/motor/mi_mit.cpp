#include "mi_mit.hpp"

MiMIT::MiMIT(uint8_t can_port, uint32_t master_id, uint32_t slave_id) :
    can_port(can_port), master_id(master_id), slave_id(slave_id) {
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);

    // MiMIT电机参数设置
    SetKt(KT * Nm_A);  // 扭矩常数 0.1 Nm/A
    SetR(0.17f * Ohm); // 相电阻约为0.17 Ohm
}

void MiMIT::SetPositionControl(float kp, float kd) {
    kp_ = kp;
    kd_ = kd;
}

void MiMIT::OnLoop() {
    Motor::OnLoop();

    if (dwt_can_send_freq.GetDT() >= 1 / can_send_freq.toFloat(Hz)) {
        dwt_can_send_freq.UpdateDT();
        send_cnt++;
        if (is_enable & is_connected) {
            if (send_cnt % 100 == 0) { // 每100次调用重新发送使能
                sendEnable();
            } else {
                sendMIT();
            }
        } else {
            sendDisable();
        }
    }
}

void MiMIT::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != master_id) return;
    if (dlc != 8) return;

    // MiMIT反馈报文检查：data[6]和data[7]应为0x00或特殊标识
    // 正常反馈: data[6]=0x00, data[7]=0x00
    // 使能确认: data[6]=0xfe, data[7]=0xef
    if (!((data[7] == 0 && data[6] == 0) || (data[6] == 0xfe && data[7] == 0xef))) {
        return; // 不是有效的反馈报文
    }

    // 解析CAN报文
    // data[0] 是电机ID
    if ((data[0] & 0x0F) != (slave_id & 0x0F)) return;
    // 解析位置（16位）
    int p_int = data[1];
    p_int <<= 8;
    p_int |= data[2];

    // 解析速度（12位）
    int v_int = data[3];
    v_int <<= 4;
    v_int |= data[4] >> 4;

    // 解析扭矩（12位）
    int t_int = data[4] & 0x0f;
    t_int <<= 8;
    t_int |= data[5];

    // 单位标准化
    angle.raw = uint_to_float(p_int, P_MIN, P_MAX, 16) * rad;
    speed.raw = uint_to_float(v_int, V_MIN, V_MAX, 12) * rad_s;
    torque.raw = uint_to_float(t_int, -I_MAX * KT * GR, I_MAX * KT * GR, 12) * Nm;
    current.raw = torque.raw / (KT * Nm_A);

    // data[6]和data[7]在MiMIT中不是温度数据
    // 如果需要温度数据，需要额外的查询命令
    temperate_mos = 0.0f * C;
    temperate_motor = 0.0f * C;

    // 调用父类公共回调函数
    Motor::callback();
}

float MiMIT::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

int MiMIT::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x - offset) * (float)((1 << bits) - 1) / span);
}

void MiMIT::sendEnable() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void MiMIT::sendDisable() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void MiMIT::sendZero() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void MiMIT::sendMIT() const {
    // 位置给定（如果不需要位置控制则设为0）
    int p_int = float_to_uint(0.0f, P_MIN, P_MAX, 16);

    // 速度给定（如果不需要速度控制则设为0）
    int v_int = float_to_uint(0.0f, V_MIN, V_MAX, 12);

    // 位置比例系数
    int kp_int = float_to_uint(kp_, KP_MIN, KP_MAX, 12);

    // 位置微分系数
    int kd_int = float_to_uint(kd_, KD_MIN, KD_MAX, 12);

    // 转矩给定值（前馈扭矩）
    float torque_cmd = is_invert ? -torque.ref.toFloat(Nm) : torque.ref.toFloat(Nm);
    int tq_int = float_to_uint(torque_cmd, -I_MAX * KT * GR, I_MAX * KT * GR, 12);

    // 按照协议打包数据
    uint8_t data[8];
    data[0] = p_int >> 8;
    data[1] = p_int & 0xff;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xf) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xff;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xf) << 4) | (tq_int >> 8);
    data[7] = tq_int & 0xff;

    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}