#include "dm4310.hpp"

#include "app.hpp"
#include "app.hpp"
#include "app.hpp"
#include "app.hpp"

DM4310::DM4310(uint8_t can_port, uint32_t master_id, uint32_t slave_id) :
    can_port(can_port), master_id(master_id), slave_id(slave_id) {
    BSP::CAN::RegisterCallback(std::bind(&DM4310::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
}

void DM4310::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id != master_id) return;
    if (dlc != 8) return;

    if ((data[0] & 0x0F) != (slave_id & 0x0F)) return;

    // 维护dt
    const float dt = dwt.GetDT();

    // 记录CAN反馈报文频率
    can_feedback_freq = 1 / dt;

    // 解析CAN报文
    err_code = (data[0] & 0xF0) >> 4;
    angle.raw = (data[1] << 8) | data[2];
    speed.raw = (data[3] << 4) | (data[4] >> 4);
    torque.raw = ((data[4] & 0x0F) << 8) | data[5];
    temperate_mos = data[6];
    temperate_motor = data[7];

    if (!is_invert) {
        angle.measure = uint_to_float(angle.raw, -P_MAX, P_MAX, 16) * rad;
        speed.measure = uint_to_float(speed.raw, -V_MAX, V_MAX, 12) * rad_s;
        torque.measure = uint_to_float(torque.raw, -T_MAX, T_MAX, 12) * Nm;
    } else {
        angle.measure = -uint_to_float(angle.raw, -P_MAX, P_MAX, 16) * rad;
        speed.measure = -uint_to_float(speed.raw, -V_MAX, V_MAX, 12) * rad_s;
        torque.measure = -uint_to_float(torque.raw, -T_MAX, T_MAX, 12) * Nm;
    }

    // 电机就绪标识
    is_ready = true;
}

float DM4310::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

uint16_t DM4310::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (uint16_t)((x - offset) * (float)((1 << bits) - 1) / span);
}

void DM4310::SetInvert(const bool is_invert) {
    this->is_invert = is_invert;
}

void DM4310::SetPIDParam(PID::param_t* pid_param) {
    pid.SetParam(pid_param);
}

void DM4310::WaitReady() {
    this->is_ready = false;
    while (this->is_ready == false) {
        sendCANDisable(); // 电机是一问一答
        BSP::OS::Delay(1);
    }
}

void DM4310::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    pid.Clear();
    torque.ref = 0;
}

void DM4310::SetAngle(Angle<> angle, UnitFloat<> speed) {
    this->angle.ref = angle;
    this->speed.ref = speed;
}

void DM4310::Update() {
    if (is_enable) {
        const Angle angle_err = angle.ref - angle.measure;
        const float speed_err = speed.ref - speed.measure;
        torque.ref = pid.CalcMIT(angle_err, speed_err);
        sendCANCmd();
    } else {
        sendCANDisable();
    }
}

void DM4310::sendCANEnable() {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void DM4310::sendCANDisable() {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
}

void DM4310::sendCANCmd() {
    if (can_cmd_cnt++ % 100 == 0) { // 每100次调用重新发送使能
        sendCANEnable();
    } else {
        const uint16_t p_des = 0; // 位置给定
        const uint16_t v_des = 0; // 速度给定
        const uint16_t Kp = 0; // 位置比例系数
        const uint16_t Kd = 0; // 位置微分系数
        uint16_t t_ff; // 转矩给定值
        if (!is_invert) {
            t_ff = float_to_uint(torque.ref, -T_MAX, T_MAX, 12);
        } else {
            t_ff = float_to_uint(-torque.ref, -T_MAX, T_MAX, 12);
        }

        uint8_t data[8];
        data[0] = p_des >> 8;
        data[1] = p_des;
        data[2] = v_des >> 4;
        data[3] = ((v_des & 0x0F) << 4) | (Kp >> 8);
        data[4] = Kp;
        data[5] = Kd >> 4;
        data[6] = ((Kd & 0x0F) << 4) | (t_ff >> 8);
        data[7] = t_ff;
        BSP::CAN::TransmitStd(can_port, slave_id, data, 8);
    }
}
